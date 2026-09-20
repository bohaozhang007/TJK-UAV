"""Robot HTTP entry point. OWL and i7 v22 adapters."""
import argparse
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit, parse_qs


class Handler(BaseHTTPRequestHandler):
    def setup(self):
        super().setup()
        self.connection.settimeout(20.)

    def dispatch(self):
        try:
            parsed = urlsplit(self.path)
            if self.command == 'POST':
                n = int(self.headers.get('Content-Length', '0'))
                if not 0 < n <= 1024 * 1024:
                    raise ValueError('invalid request size')
                data = json.loads(self.rfile.read(n))
                if not isinstance(data, dict):
                    raise ValueError('request must be an object')
            else:
                data = {k: v[-1] for k, v in parse_qs(parsed.query).items()}
            result = self.server.controller.handle_http(self.command, parsed.path, data,
                local_operator=self.client_address[0] in ('127.0.0.1', '::1'))
            status = 200
        except Exception as exc:
            status = getattr(exc, 'code', 400)
            result = dict(ok=False, error=str(exc), **{k: getattr(exc, k)
                for k in ('error_code', 'retryable', 'timings') if hasattr(exc, k)})
        body = json.dumps(result, allow_nan=False).encode()
        try:
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    do_GET = do_POST = dispatch


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot', choices=['owl_ego', 'i7'], default='owl_ego')
    parser.add_argument('--config', default=None)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8765)
    args = parser.parse_args()
    if args.robot == 'i7':
        from .controllers.i7 import I7Controller as Controller
    else:
        from .controllers.owl_ego import OwlEgoController as Controller
    ThreadingHTTPServer.request_queue_size = 64
    with ThreadingHTTPServer((args.host, args.port), Handler) as server:
        server.controller = Controller(config_path=args.config)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            server.controller.close()


if __name__ == '__main__':
    main()
