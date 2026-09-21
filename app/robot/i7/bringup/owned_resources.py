"""Track ROS registrations and listening sockets by descendant process ownership."""
import os
from pathlib import Path
import socket
import threading
from urllib.parse import urlsplit, urlunsplit
from xmlrpc.client import Error as XmlRpcError, ServerProxy, Transport


class TimeoutTransport(Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = .3
        return connection


def rpc(uri, method, *args):
    with ServerProxy(uri, transport=TimeoutTransport()) as proxy:
        code, message, value = getattr(proxy, method)(*args)
    if code != 1:
        raise RuntimeError(message)
    return value


def local_uri(uri):
    parts = urlsplit(uri)
    if parts.hostname in {socket.gethostname().lower(), socket.getfqdn().lower(), 'localhost'}:
        return urlunsplit((parts.scheme, '127.0.0.1:'+str(parts.port), parts.path, '', ''))
    return uri


def listeners():
    result = {}
    for name in ('tcp', 'tcp6', 'udp', 'udp6'):
        for line in Path('/proc/net', name).read_text().splitlines()[1:]:
            fields = line.split()
            if name.startswith('tcp') and fields[3] != '0A':
                continue
            result[fields[9]] = (name, int(fields[1].split(':')[1], 16))
    return result


class OwnedResources:
    def __init__(self, processes):
        self.processes = processes
        self.master = local_uri(os.environ.get('ROS_MASTER_URI', 'http://localhost:11311'))
        self.nodes = {}
        self.sockets = {}
        self.done = threading.Event()
        self.thread = threading.Thread(target=self.monitor, daemon=True)

    def start(self):
        self.thread.start()

    def monitor(self):
        while not self.done.is_set():
            try:
                self.sample()
            except (OSError, RuntimeError, XmlRpcError):
                pass
            self.done.wait(.25)

    def sample(self):
        owned = self.processes()
        inodes = set()
        for pid in owned:
            try:
                for fd in Path('/proc', str(pid), 'fd').iterdir():
                    try:
                        target = os.readlink(fd)
                    except FileNotFoundError:
                        continue
                    if target.startswith('socket:['):
                        inodes.add(target[8:-1])
            except FileNotFoundError:
                continue
        self.sockets.update({key: value for key, value in listeners().items() if key in inodes})
        state = rpc(self.master, 'getSystemState', '/i7_owned_resources')
        names = {n for group in state for _, nodes in group for n in nodes}
        local_hosts = {socket.gethostname().lower(), socket.getfqdn().lower(), 'localhost', '127.0.0.1', '::1'}
        local_hosts.update(info[4][0] for info in socket.getaddrinfo(socket.gethostname(), None))
        for name in names:
            if self.done.is_set():
                return
            try:
                uri = rpc(self.master, 'lookupNode', '/i7_owned_resources', name)
                if urlsplit(uri).hostname not in local_hosts:
                    continue
                pid = rpc(local_uri(uri), 'getPid', '/i7_owned_resources')
                current = self.processes().get(pid)
                if pid not in owned or current is None or current[1] != owned[pid][1]:
                    continue
                services = {topic: rpc(self.master, 'lookupService', '/i7_owned_resources', topic)
                            for topic, nodes in state[2] if name in nodes}
                self.nodes[name] = (uri, services)
            except (OSError, RuntimeError, XmlRpcError):
                continue

    def cleanup(self):
        self.done.set()
        self.thread.join(timeout=2.)
        if self.thread.is_alive():
            print('Cleanup incomplete: ROS ownership monitor did not stop.', flush=True)
            return False
        remaining = {key: value for key, value in listeners().items() if key in self.sockets}
        ok = not remaining
        if remaining:
            print('Cleanup incomplete: owned sockets remain: '+str(remaining), flush=True)
        if not self.nodes:
            return ok
        try:
            state = rpc(self.master, 'getSystemState', '/i7_owned_resources')
        except ConnectionRefusedError:
            return ok  # The owned master exited as well.
        except (OSError, RuntimeError, XmlRpcError) as exc:
            print('Cannot verify ROS cleanup: '+str(exc), flush=True)
            return False
        for name, (uri, services) in self.nodes.items():
            if not any(name in nodes for group in state for _, nodes in group):
                continue
            try:
                current = rpc(self.master, 'lookupNode', '/i7_owned_resources', name)
                if current != uri:
                    continue  # Another launcher now owns this name.
                for group, method in ((state[0], 'unregisterPublisher'), (state[1], 'unregisterSubscriber')):
                    for topic, nodes in group:
                        if name in nodes:
                            rpc(self.master, method, name, topic, uri)
                for topic, nodes in state[2]:
                    if name in nodes and topic in services:
                        rpc(self.master, 'unregisterService', name, topic, services[topic])
                print('Cleaned owned ROS registration: '+name, flush=True)
            except (OSError, RuntimeError, XmlRpcError) as exc:
                print('ROS cleanup failed for '+name+': '+str(exc), flush=True)
                ok = False
        try:
            state = rpc(self.master, 'getSystemState', '/i7_owned_resources')
            for name, (uri, _) in self.nodes.items():
                if (any(name in nodes for group in state for _, nodes in group)
                        and rpc(self.master, 'lookupNode', '/i7_owned_resources', name) == uri):
                    print('Cleanup incomplete: owned ROS registration remains: '+name, flush=True)
                    ok = False
        except ConnectionRefusedError:
            pass
        except (OSError, RuntimeError, XmlRpcError) as exc:
            print('Cannot verify final ROS registrations: '+str(exc), flush=True)
            ok = False
        return ok
