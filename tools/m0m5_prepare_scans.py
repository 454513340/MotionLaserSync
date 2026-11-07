#!/usr/bin/env python3
"""Read-only, strict archive5D v5 SCAN-v3 converter for portable M0-M5 CLI.
Source: CompactScanFormat / ScanArchive5D. Enforces header/body CRC, pose,
index bounds, duplicate index, timestamps and full record accounting.
Each source valid sample is transformed ONCE from sensor (X,0,Z) to MCS.
An optional deterministic stride of raw laser indexes is a computation
sampling parameter, not an estimate of full-resolution coverage.
"""
import argparse,hashlib,json,pathlib,struct,zlib
import numpy as np
PD=np.dtype([('index','<u4'),('x','<f8'),('z','<f8')])
OUT=np.dtype([('frame','<u4'),('x','<f8'),('y','<f8'),('z','<f8')])
BASE=pathlib.Path(__file__).resolve().parents[1]

def unwrap_sensor_timestamp(raw,previous_raw,previous_unwrapped,offset,ordinal,previous_ordinal,frame):
    """Conservatively unwrap an observed 128-second hardware timestamp rollover.

    A wrap is accepted only at 127.5..128 -> 0..0.5, with contiguous
    source ordinal and a plausible 0..0.1 second inter-frame interval.
    Other backward steps and unphysically large gaps remain hard errors.
    This repairs archive *time interpretation*, never frame indices or points.
    """
    if not np.isfinite(raw) or raw<0 or raw>=128.0:
        raise ValueError(f'Invalid raw sensor timestamp frame={frame}: {raw}')
    unwrapped=raw+offset
    wrap=False
    if unwrapped<=previous_unwrapped:
        valid=(previous_raw is not None and 127.5<previous_raw<128.0 and 0<=raw<0.5
               and ordinal==previous_ordinal+1 and 0<raw+128.0-previous_raw<0.1)
        if not valid:
            raise ValueError(f'Non-rollover sensor time reversal at frame={frame}: {raw} after {previous_raw}')
        offset+=128.0;unwrapped=raw+offset;wrap=True
    if np.isfinite(previous_unwrapped) and unwrapped-previous_unwrapped>0.1:
        raise ValueError(f'Unexpected sensor clock gap at frame={frame}: {unwrapped-previous_unwrapped:.6f}s')
    return unwrapped,offset,wrap
def prep(path,outdir,stride,model,save_m4=True,m4_profile_step=None):
    content=path.read_bytes()
    h=content[:256]
    if len(h)!=256 or h[:4]!=b'SCAN' or struct.unpack_from('<I',h,4)[0]!=3:raise ValueError('SCAN v3 header required')
    if zlib.crc32(h[:252])!=struct.unpack_from('<I',h,252)[0]:raise ValueError('Bad header CRC')
    count=struct.unpack_from('<Q',h,8)[0]; nominal=struct.unpack_from('<H',h,28)[0]
    if count==0 or count>1000000 or nominal!=1280:raise ValueError('Unexpected profile count or sampleCount')
    rawlen,clen=struct.unpack_from('<QQ',h,116)
    if h[112]==3:
        packed=content[256:]
        if len(packed)!=clen or int.from_bytes(packed[:4],'big')!=rawlen:raise ValueError('Wrong deflate payload length')
        payload=zlib.decompress(packed[4:])
    elif h[112]==0:
        payload=content[256:]
    else:raise ValueError('Unsupported archive codec')
    if len(payload)!=rawlen:raise ValueError('Uncompressed archive byte length mismatch')
    if zlib.crc32(payload[:-4])!=struct.unpack_from('<I',payload,len(payload)-4)[0]:raise ValueError('Bad payload CRC')
    magic,v,n=struct.unpack_from('<IIQ',payload,0)
    if (magic,v,n)!=(0x34443541,5,count):raise ValueError('Expected Archive5D v5 + frame count')
    p=16;prevtime=-float('inf');prev_raw_time=None;time_offset=0.0;time_wraps=[]
    session=None;prevord=-1;total=0;written=0;m4frames=[]
    out=outdir/(path.stem+'.m5s1')
    with out.open('wb') as f:
        f.write(b'M5S1'+struct.pack('<QQ',count,0))
        for frame in range(count):
            if p+492>len(payload)-4:raise ValueError('Truncated profile fixed record')
            sess,ordinal,host,tm=struct.unpack_from('<QQqq',payload,p)
            profile_index,t,feedback,latch,command,gauge,nn=struct.unpack_from('<IIIIIII',payload,p+32)
            timestamp=struct.unpack_from('<d',payload,p+60)[0]
            q=np.frombuffer(payload,dtype='<f8',count=5,offset=p+92)
            T=np.frombuffer(payload,dtype='<f8',count=16,offset=p+228).reshape(4,4)
            if nn>1280 or p+492+nn*20>len(payload)-4:raise ValueError('Bad profile length')
            data=np.frombuffer(payload,dtype=PD,count=nn,offset=p+492)
            p+=492+nn*20
            if session is None:session=sess
            if session!=sess or ordinal<=prevord:
                raise ValueError('Session/ordinal sequence not monotonic')
            unwrapped,time_offset,wrapped=unwrap_sensor_timestamp(
                timestamp,prev_raw_time,prevtime,time_offset,ordinal,prevord,frame)
            if wrapped:
                time_wraps.append({'frame':frame,'ordinal':int(ordinal),'previous_raw_s':float(prev_raw_time),
                                   'raw_s':float(timestamp),'unwrapped_s':float(unwrapped)})
            prevord=ordinal;prevtime=unwrapped;prev_raw_time=timestamp
            if not np.all(np.isfinite(T)) or not np.all(np.isfinite(q)) or not np.allclose(T[3],[0,0,0,1],atol=1e-6) or not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-3) or abs(np.linalg.det(T[:3,:3])-1)>1e-3:raise ValueError('Bad Frame5D rigid transform')
            idx=data['index']
            if len(idx) and (idx.max()>=1280 or len(np.unique(idx))!=nn):raise ValueError('Invalid/duplicate sample index')
            if not np.all(np.isfinite(data['x'])) or not np.all(np.isfinite(data['z'])):raise ValueError('Nonfinite raw point')
            total+=nn
            sel=(idx%stride==0)
            pts=data[sel];num=len(pts);written+=num
            if num:
                xyz=np.empty(num,dtype=OUT)
                xyz['frame']=frame+1
                x=pts['x'];z=pts['z']
                xyz['x']=T[0,0]*x+T[0,2]*z+T[0,3]
                xyz['y']=T[1,0]*x+T[1,2]*z+T[1,3]
                xyz['z']=T[2,0]*x+T[2,2]*z+T[2,3]
                if not (np.isfinite(xyz['x']).all() and np.isfinite(xyz['y']).all() and np.isfinite(xyz['z']).all()):raise ValueError('Nonfinite machine point')
                f.write(xyz.tobytes())
            # For M4 use independent scan frames, NOT downsampled laser columns.
            # Uniform profile selection retains real original sample indexes.
            if save_m4 and frame%(m4_profile_step if m4_profile_step else max(1,int(count)//35))==0:
                m4frames.append((int(ordinal),float(unwrapped),T.copy(),data.copy()))
        if p!=len(payload)-4:raise ValueError('Trailing SCAN bytes')
        f.seek(12);f.write(struct.pack('<Q',written))
    if save_m4:
        ff=outdir/(path.stem+'.m4s1')
        with ff.open('wb') as g:
            g.write(b'M4S1'+struct.pack('<Q',len(m4frames)))
            for ordinal,t,T,data in m4frames:
                g.write(struct.pack('<Qd',ordinal,t))
                g.write(T.astype('<f8').tobytes())
                g.write(struct.pack('<I',len(data)))
                g.write(data.tobytes())
    return dict(scan=path.name,sha256=hashlib.sha256(content).hexdigest(),crc_valid=True,frames=int(count),valid_points=int(total),exported_points=int(written),stride=int(stride),m4_frames=len(m4frames),session_id=int(session),time_rollovers_128s=time_wraps,unwrapped_last_sensor_time_s=float(prevtime),m5_binary=out.name,m4_binary=path.stem+'.m4s1')

def main():
    a=argparse.ArgumentParser(description=__doc__)
    a.add_argument('--scan',type=pathlib.Path,nargs='+',required=True)
    a.add_argument('--step',type=pathlib.Path,required=True)
    a.add_argument('--output',type=pathlib.Path,required=True)
    a.add_argument('--stride',type=int,default=1)
    a.add_argument('--m4-profile-step',type=int,default=None,help='Select every Nth complete profile for M3/M4, 1 means all; default about 36')
    q=a.parse_args()
    if q.stride<1 or q.stride>1280:raise ValueError('stride outside [1,1280]')
    if q.m4_profile_step is not None and q.m4_profile_step<1:raise ValueError('m4-profile-step must be positive')
    expected='798569b0f411d68669c5fa076c837fd33755f602d48b292dba2e9540819839a2'
    actual=hashlib.sha256(q.step.read_bytes()).hexdigest()
    if actual!=expected:raise ValueError('STEP changed since derived mesh! Do not reuse previous OBJ or pose; regenerate via OCCT.')
    q.output.mkdir(parents=True,exist_ok=True)
    report=[prep(path,q.output,q.stride,None,m4_profile_step=q.m4_profile_step) for path in q.scan]
    with (q.output/'source_manifest.json').open('w',encoding='utf-8') as f:
        json.dump({'cad_step_sha256':actual,'mesh':'6061_sensor_connector_NEW_2026.obj','coordinate':'MCS mm, exposure Frame5D applied once','per_index_stride':q.stride,'sessions':report},f,indent=2,ensure_ascii=False)
    for r in report:print(r['scan'],r['frames'],r['valid_points'],'selected',r['exported_points'])
if __name__=='__main__':main()
