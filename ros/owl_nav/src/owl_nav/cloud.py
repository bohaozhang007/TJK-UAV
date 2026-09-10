"""Validate PointCloud2 storage before forwarding it to the planner."""
import numpy as np

def validate(message):
    if message.width<=0 or message.height<=0 or message.point_step<=0:
        raise ValueError('empty obstacle cloud')
    if message.row_step<message.width*message.point_step or len(message.data)!=message.row_step*message.height:
        raise ValueError('invalid obstacle cloud buffer')
    fields={f.name:f for f in message.fields}
    valid=np.ones((message.height,message.width),dtype=bool)
    for name in ('x','y','z'):
        f=fields.get(name)
        if f is None or f.datatype != 7 or f.count!=1:
            raise ValueError('invalid obstacle cloud field '+name)
        dtype=np.dtype(('>' if message.is_bigendian else '<')+'f4')
        if f.offset<0 or f.offset+dtype.itemsize>message.point_step:
            raise ValueError('invalid obstacle cloud offset')
        axis=np.ndarray((message.height,message.width),dtype=dtype,buffer=message.data,
                        offset=f.offset,strides=(message.row_step,message.point_step))
        if np.isinf(axis).any():raise ValueError('infinite obstacle cloud coordinate')
        valid &= np.isfinite(axis)
    if not valid.any():raise ValueError('no finite obstacle points')
    return int(valid.sum())
