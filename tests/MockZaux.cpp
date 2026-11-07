#include "MockZaux.h"
#include <algorithm>
#include <chrono>
#include <thread>
MockZaux mock;
void MockZaux::armBlock(const char* name) {
    std::lock_guard<std::mutex> lock(hookMutex);blockName=name;blockEntered=false;releaseBlocked=false;
}
bool MockZaux::waitBlocked() {
    std::unique_lock<std::mutex> lock(hookMutex);
    return hookWake.wait_for(lock,std::chrono::seconds(2),[&]{return blockEntered;});
}
void MockZaux::releaseBlock() {
    {std::lock_guard<std::mutex> lock(hookMutex);releaseBlocked=true;}hookWake.notify_all();
}
void MockZaux::beforeCall(const char* name) {
    std::unique_lock<std::mutex> lock(hookMutex);
    if (blockName!=name) return;
    blockName.clear();blockEntered=true;hookWake.notify_all();
    hookWake.wait(lock,[&]{return releaseBlocked;});
}
void MockZaux::reset() {
    std::lock_guard<std::mutex> lock(mutex);
    {std::lock_guard<std::mutex> h(hookMutex);blockName.clear();blockEntered=false;releaseBlocked=false;}
    buffered=0;accountBuffered=false;failHomeRestore=false;
    inFlight=0; maxInFlight=0; callCount=0; delayUs=0;
    dpos.fill(0); mpos.fill(0); encoder.fill(0); idle.fill(-1); homeStatus.fill(0); op.fill(0); table.fill(0);
    table[330]=96;table[337]=270909;table[338]=6;table[339]=5;table[342]=1;
    table[343]=31;table[344]=4;table[345]=31;table[346]=0;table[347]=0;table[348]=14;table[349]=104;table[350]=14;table[351]=1;
    table[362]=1316;table[363]=0;table[364]=0;table[354]=321058;table[355]=200;table[356]=200;table[357]=0;table[358]=0;table[359]=0;table[360]=0;table[361]=0;table[365]=1;table[366]=104;table[367]=1;table[368]=104;table[369]=0;
    status.fill(0);failSp=false;failPause=false;failResume=false;failInterp=false;
    calls.clear();holdMotion=false;failAck=false;failAllTableReads=false;suppressPwmConfigAck=false; ignoreHomeMaskWrites=false;
    dataReads=0;failDataRead=0;opens=0;closes=0;
}
namespace {
struct CallGuard {
    std::unique_lock<std::mutex> lock;
    explicit CallGuard(const char* name) {
        const int n=mock.inFlight.fetch_add(1)+1;
        int old=mock.maxInFlight.load();
        while(old<n && !mock.maxInFlight.compare_exchange_weak(old,n)) {}
        if(mock.delayUs.load()>0) std::this_thread::sleep_for(std::chrono::microseconds(mock.delayUs.load()));
        mock.beforeCall(name);
        lock=std::unique_lock<std::mutex>(mock.mutex);
        ++mock.callCount;
        mock.calls.push_back({name,{},{},0,std::this_thread::get_id()});
    }
    ~CallGuard(){ --mock.inFlight; }
};
void setMove(int n,int* axes,float* pos) {
    auto& c=mock.calls.back();c.axes.assign(axes,axes+n);c.target.assign(pos,pos+n);
    for(int i=0;i<n;++i) {mock.dpos[axes[i]]=pos[i];mock.mpos[axes[i]]=pos[i];mock.encoder[axes[i]]=pos[i];mock.idle[axes[i]]=mock.holdMotion?0:-1;}
}
}
int32 __stdcall ZAux_Close(ZMC_HANDLE handle) {
    CallGuard call("ZAux_Close");
    ++mock.closes;
    return 0;
}

int32 __stdcall ZAux_Direct_Base(ZMC_HANDLE handle, int imaxaxises, int *piAxislist) {
    CallGuard call("ZAux_Direct_Base");
    mock.calls.back().axes.assign(piAxislist,piAxislist+imaxaxises);
    return 0;
}

int32 __stdcall ZAux_Direct_CancelAxisList(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, int imode) {
    CallGuard call("ZAux_Direct_CancelAxisList");
    mock.calls.back().axes.assign(piAxislist,piAxislist+imaxaxises);mock.calls.back().mode=imode;
    for(int i=0;i<imaxaxises;++i) mock.idle[piAxislist[i]]=-1;
    return 0;
}

int32 __stdcall ZAux_Direct_GetAccel(ZMC_HANDLE handle, int iaxis, float *pfValue) {
    CallGuard call("ZAux_Direct_GetAccel");
    *pfValue=100.f;
    return 0;
}

int32 __stdcall ZAux_Direct_GetAxisStatus(ZMC_HANDLE handle, int iaxis, int *piValue) {
    CallGuard call("ZAux_Direct_GetAxisStatus");
    *piValue=mock.status[iaxis];
    return 0;
}

int32 __stdcall ZAux_Direct_GetCreep(ZMC_HANDLE handle, int iaxis, float *pfValue) {
    CallGuard call("ZAux_Direct_GetCreep");
    *pfValue=100.f;
    return 0;
}

int32 __stdcall ZAux_Direct_GetDecel(ZMC_HANDLE handle, int iaxis, float *pfValue) {
    CallGuard call("ZAux_Direct_GetDecel");
    *pfValue=100.f;
    return 0;
}

int32 __stdcall ZAux_Direct_GetDpos(ZMC_HANDLE handle, int iaxis, float *pfValue) {
    CallGuard call("ZAux_Direct_GetDpos");
    *pfValue=mock.dpos[iaxis];
    return 0;
}

int32 __stdcall ZAux_Direct_GetEncoder(ZMC_HANDLE handle, int iaxis, float *pfValue) {
    CallGuard call("ZAux_Direct_GetEncoder");
    *pfValue=mock.encoder[iaxis];
    return 0;
}

int32 __stdcall ZAux_Direct_GetFsLimit(ZMC_HANDLE handle, int iaxis, float *pfValue) {
    CallGuard call("ZAux_Direct_GetFsLimit");
    *pfValue=10000.f;
    return 0;
}

int32 __stdcall ZAux_Direct_GetIfIdle(ZMC_HANDLE handle, int iaxis, int *piValue) {
    CallGuard call("ZAux_Direct_GetIfIdle");
    *piValue=mock.idle[iaxis];
    return 0;
}

int32 __stdcall ZAux_Direct_GetLspeed(ZMC_HANDLE handle, int iaxis, float *pfValue) {
    CallGuard call("ZAux_Direct_GetLspeed");
    *pfValue=100.f;
    return 0;
}

int32 __stdcall ZAux_Direct_GetMovesBuffered(ZMC_HANDLE handle, int iaxis, int *piValue) {
    CallGuard call("ZAux_Direct_GetMovesBuffered");
    *piValue=mock.buffered;
    return 0;
}

int32 __stdcall ZAux_Direct_GetMpos(ZMC_HANDLE handle, int iaxis, float *pfValue) {
    CallGuard call("ZAux_Direct_GetMpos");
    *pfValue=mock.mpos[iaxis];
    return 0;
}

int32 __stdcall ZAux_Direct_GetHomeStatus(ZMC_HANDLE handle, uint32 iaxis, uint32 *homestatus) {
    CallGuard call("ZAux_Direct_GetHomeStatus");
    *homestatus=mock.homeStatus[iaxis];
    return 0;
}

int32 __stdcall ZAux_Direct_GetDatumIn(ZMC_HANDLE handle, int iaxis, int *piValue) {
    CallGuard call("ZAux_Direct_GetDatumIn");
    *piValue = 17 + iaxis;
    return 0;
}

int32 __stdcall ZAux_Direct_GetIn(ZMC_HANDLE handle, int ionum, uint32 *piValue) {
    CallGuard call("ZAux_Direct_GetIn");
    *piValue = 0;
    return 0;
}

int32 __stdcall ZAux_Direct_GetPwmDuty(ZMC_HANDLE handle, int ionum, float *pfValue) {
    CallGuard call("ZAux_Direct_GetPwmDuty");
    (void)handle; (void)ionum;
    if (pfValue) *pfValue = 0.0f;
    return 0;
}
int32 __stdcall ZAux_Direct_GetOp(ZMC_HANDLE handle, int ionum, uint32 *piValue) {
    CallGuard call("ZAux_Direct_GetOp");
    *piValue=mock.op[ionum];
    return 0;
}

int32 __stdcall ZAux_Direct_GetRemain_LineBuffer(ZMC_HANDLE handle, int iaxis,int * piValue) {
    CallGuard call("ZAux_Direct_GetRemain_LineBuffer");
    *piValue=100-mock.buffered;
    return 0;
}

int32 __stdcall ZAux_Direct_GetRsLimit(ZMC_HANDLE handle, int iaxis, float *pfValue) {
    CallGuard call("ZAux_Direct_GetRsLimit");
    *pfValue=-10000.f;
    return 0;
}

int32 __stdcall ZAux_Direct_GetSpeed(ZMC_HANDLE handle, int iaxis, float *pfValue) {
    CallGuard call("ZAux_Direct_GetSpeed");
    *pfValue=100.f;
    return 0;
}

int32 __stdcall ZAux_Direct_GetSramp(ZMC_HANDLE handle, int iaxis, float *pfValue) {
    CallGuard call("ZAux_Direct_GetSramp");
    *pfValue=100.f;
    return 0;
}

int32 __stdcall ZAux_Direct_GetTable(ZMC_HANDLE handle, int tabstart, int numes, float *pfValue) {
    CallGuard call("ZAux_Direct_GetTable");
    if(mock.failAllTableReads) return -91;
    if(tabstart<0 || numes<0 || tabstart+numes>1600) return -92;
    if(tabstart>=350 && ++mock.dataReads==mock.failDataRead) return -93;
    std::copy_n(mock.table.data()+tabstart,numes,pfValue);
    return 0;
}

int32 __stdcall ZAux_Direct_GetUnits(ZMC_HANDLE handle, int iaxis, float *pfValue) {
    CallGuard call("ZAux_Direct_GetUnits");
    *pfValue=100.f;
    return 0;
}

int32 __stdcall ZAux_Direct_MoveAbs(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float *pfDisancelist) {
    CallGuard call("ZAux_Direct_MoveAbs");
    setMove(imaxaxises,piAxislist,pfDisancelist);
    if(mock.accountBuffered && imaxaxises==3) ++mock.buffered;
    return 0;
}

int32 __stdcall ZAux_Direct_MoveCirc2(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fmid1, float fmid2, float fend1, float fend2) {
    CallGuard call("ZAux_Direct_MoveCirc2");
    float pos[2]={fend1,fend2};setMove(imaxaxises,piAxislist,pos);
    return 0;
}

int32 __stdcall ZAux_Direct_MoveCirc2Abs(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fmid1, float fmid2, float fend1, float fend2) {
    CallGuard call("ZAux_Direct_MoveCirc2Abs");
    float pos[2]={fend1,fend2};setMove(imaxaxises,piAxislist,pos);
    return 0;
}

int32 __stdcall ZAux_Direct_SetAccel(ZMC_HANDLE handle, int iaxis, float fValue) {
    CallGuard call("ZAux_Direct_SetAccel");
    
    return 0;
}

int32 __stdcall ZAux_Direct_SetCreep(ZMC_HANDLE handle, int iaxis, float fValue) {
    CallGuard call("ZAux_Direct_SetCreep");
    
    return 0;
}

int32 __stdcall ZAux_Direct_SetDecel(ZMC_HANDLE handle, int iaxis, float fValue) {
    CallGuard call("ZAux_Direct_SetDecel");
    
    return 0;
}

int32 __stdcall ZAux_Direct_SetDpos(ZMC_HANDLE handle, int iaxis, float fValue) {
    CallGuard call("ZAux_Direct_SetDpos");
    mock.dpos[iaxis]=fValue;
    return 0;
}

int32 __stdcall ZAux_Direct_SetMpos(ZMC_HANDLE handle, int iaxis, float fValue) {
    CallGuard call("ZAux_Direct_SetMpos");
    mock.mpos[iaxis]=fValue;
    return 0;
}

int32 __stdcall ZAux_Direct_SetFsLimit(ZMC_HANDLE handle, int iaxis, float fValue) {
    CallGuard call("ZAux_Direct_SetFsLimit");
    if(mock.failHomeRestore && fValue==10000.f) return -97;
    
    return 0;
}

int32 __stdcall ZAux_Direct_SetLspeed(ZMC_HANDLE handle, int iaxis, float fValue) {
    CallGuard call("ZAux_Direct_SetLspeed");
    
    return 0;
}

int32 __stdcall ZAux_Direct_SetMerge(ZMC_HANDLE handle, int iaxis, int iValue) {
    CallGuard call("ZAux_Direct_SetMerge");
    
    return 0;
}

int32 __stdcall ZAux_Direct_SetMovemark(ZMC_HANDLE handle, int iaxis, int iValue) {
    CallGuard call("ZAux_Direct_SetMovemark");
    
    return 0;
}

int32 __stdcall ZAux_Direct_SetOp(ZMC_HANDLE handle, int ionum, uint32 iValue) {
    CallGuard call("ZAux_Direct_SetOp");
    mock.op[ionum]=iValue;
    return 0;
}

int32 __stdcall ZAux_Direct_SetRsLimit(ZMC_HANDLE handle, int iaxis, float fValue) {
    CallGuard call("ZAux_Direct_SetRsLimit");
    
    return 0;
}

int32 __stdcall ZAux_Direct_SetSpeed(ZMC_HANDLE handle, int iaxis, float fValue) {
    CallGuard call("ZAux_Direct_SetSpeed");
    
    return 0;
}

int32 __stdcall ZAux_Direct_SetSramp(ZMC_HANDLE handle, int iaxis, float fValue) {
    CallGuard call("ZAux_Direct_SetSramp");
    
    return 0;
}

int32 __stdcall ZAux_Direct_SetTable(ZMC_HANDLE handle,int tabstart, int numes, float *pfValue) {
    CallGuard call("ZAux_Direct_SetTable");
    mock.calls.back().axes={tabstart};mock.calls.back().target.assign(pfValue,pfValue+numes);
    if(tabstart<0 || numes<0 || tabstart+numes>1600) return -92;
    if(tabstart==357 && mock.ignoreHomeMaskWrites) return 0;
    std::copy_n(pfValue,numes,mock.table.data()+tabstart);
    if(tabstart==332 && mock.failAck) return -94;
    // V32.13 protocol mock: freq request is committed by TABLE[359], not TABLE[355].
    // This is only an ACK/state model; there is no PWM, REGIST FIFO, or BASIC interpreter.
    if(tabstart==359 && numes>=1 && !mock.suppressPwmConfigAck && mock.table[334]==0 &&
       (mock.table[358]==0 || mock.table[358]==5)) {
        const float f=mock.table[355],seq=mock.table[359];
        const int mode=int(mock.table[365]), edge=int(mock.table[366]);
        if(f<25 || f>2000 || f!=static_cast<int>(f) || seq<1 || seq>16777214 ||
           !(mode>=1 && mode<=3 && (mode==1?edge==104:(edge==114 || edge==115)))) {
            mock.table[361]=10;mock.table[358]=5;
        } else {
            mock.table[356]=f;mock.table[360]=seq;mock.table[361]=0;mock.table[358]=0;
            mock.table[367]=mode;mock.table[368]=edge;
            mock.table[349]=edge;mock.table[347]=(mode==1?0:1);
            mock.table[350]=(mode==1?14:0);
        }
    }
    if(tabstart==334 && numes>=1) {
        const int mode=static_cast<int>(pfValue[0]);
        if(mode==0) {mock.table[358]=0;mock.table[369]=0;} // simplified STOP
        if(mode>=1 && mode<=3) mock.table[358]=mode;
    }
    // Protocol test double only: emulate disabled-session ACK and ONE probe.
    // No BASIC interpreter, servo dynamics or exposure timing is simulated here.
    if(tabstart==340 && mock.table[334]==0){
        mock.table[331]=mock.table[332]=mock.table[333]=mock.table[335]=mock.table[336]=0;
        mock.table[341]=mock.table[340];
    }
    if(tabstart==334 && pfValue[0]==2){
        float* record=mock.table.data()+370;record[0]=0;
        record[1]=mock.dpos[1];record[2]=mock.dpos[2];record[3]=mock.dpos[3];record[4]=mock.mpos[4];record[5]=mock.mpos[5];
        mock.table[331]=1;mock.table[335]=1;mock.table[336]=1;mock.table[334]=2;mock.table[369]=1;
    }
    return 0;
}

int32 __stdcall ZAux_Direct_SetUnits(ZMC_HANDLE handle, int iaxis, float fValue) {
    CallGuard call("ZAux_Direct_SetUnits");
    
    return 0;
}

int32 __stdcall ZAux_Direct_Single_Cancel(ZMC_HANDLE handle, int iaxis, int imode) {
    CallGuard call("ZAux_Direct_Single_Cancel");
    mock.calls.back().axes={iaxis};mock.calls.back().mode=imode;mock.idle[iaxis]=-1;
    return 0;
}

int32 __stdcall ZAux_Direct_Single_Datum(ZMC_HANDLE handle, int iaxis, int imode) {
    CallGuard call("ZAux_Direct_Single_Datum");
    float pos=0;setMove(1,&iaxis,&pos);
    mock.homeStatus[iaxis]=1;
    return 0;
}

int32 __stdcall ZAux_Direct_Single_Move(ZMC_HANDLE handle, int iaxis, float fdistance) {
    CallGuard call("ZAux_Direct_Single_Move");
    float pos=mock.dpos[iaxis]+fdistance;setMove(1,&iaxis,&pos);
    return 0;
}

int32 __stdcall ZAux_Direct_Single_MoveAbs(ZMC_HANDLE handle, int iaxis, float fdistance) {
    CallGuard call("ZAux_Direct_Single_MoveAbs");
    setMove(1,&iaxis,&fdistance);
    return 0;
}

int32 __stdcall ZAux_Direct_Single_Vmove(ZMC_HANDLE handle, int iaxis, int idir) {
    CallGuard call("ZAux_Direct_Single_Vmove");
    mock.idle[iaxis]=0;mock.calls.back().axes={iaxis};
    return 0;
}

int32 __stdcall ZAux_OpenEth(char *ipaddr, ZMC_HANDLE * phandle) {
    CallGuard call("ZAux_OpenEth");
    ++mock.opens; *phandle=static_cast<void*>(&mock);
    return 0;
}


int32 __stdcall ZAux_Direct_SetCornerMode(ZMC_HANDLE,int axis,int value){CallGuard c("ZAux_Direct_SetCornerMode");mock.calls.back().axes={axis};mock.calls.back().mode=value;return 0;}
int32 __stdcall ZAux_Direct_SetDecelAngle(ZMC_HANDLE,int axis,float value){CallGuard c("ZAux_Direct_SetDecelAngle");mock.calls.back().axes={axis};mock.calls.back().target={value};return 0;}
int32 __stdcall ZAux_Direct_SetStopAngle(ZMC_HANDLE,int axis,float value){CallGuard c("ZAux_Direct_SetStopAngle");mock.calls.back().axes={axis};mock.calls.back().target={value};return 0;}
int32 __stdcall ZAux_Direct_SetInterpFactor(ZMC_HANDLE,int axis,int value){CallGuard c("ZAux_Direct_SetInterpFactor");if(mock.failInterp)return -99;mock.calls.back().axes={axis};mock.calls.back().mode=value;return 0;}
int32 __stdcall ZAux_Direct_GetRemain_Buffer(ZMC_HANDLE,int axis,int* value){CallGuard c("ZAux_Direct_GetRemain_Buffer");*value=(100-mock.buffered)/5;return 0;}
int32 __stdcall ZAux_Direct_SetStartMoveSpeed(ZMC_HANDLE,int axis,float value){CallGuard c("ZAux_Direct_SetStartMoveSpeed");mock.calls.back().axes={axis};mock.calls.back().target={value};return 0;}
int32 __stdcall ZAux_Direct_SetForceSpeed(ZMC_HANDLE,int axis,float value){CallGuard c("ZAux_Direct_SetForceSpeed");mock.calls.back().axes={axis};mock.calls.back().target={value};return 0;}
int32 __stdcall ZAux_Direct_SetEndMoveSpeed(ZMC_HANDLE,int axis,float value){CallGuard c("ZAux_Direct_SetEndMoveSpeed");mock.calls.back().axes={axis};mock.calls.back().target={value};return 0;}
int32 __stdcall ZAux_Direct_MoveAbsSp(ZMC_HANDLE,int n,int* axes,float* target){CallGuard c("ZAux_Direct_MoveAbsSp");if(mock.failSp)return -99;setMove(n,axes,target);if(mock.accountBuffered)++mock.buffered;return 0;}
int32 __stdcall ZAux_Direct_MovePause(ZMC_HANDLE,int axis,int mode){CallGuard c("ZAux_Direct_MovePause");if(mock.failPause)return -99;mock.calls.back().axes={axis};mock.calls.back().mode=mode;return 0;}
int32 __stdcall ZAux_Direct_MoveResume(ZMC_HANDLE,int axis){CallGuard c("ZAux_Direct_MoveResume");if(mock.failResume)return -99;mock.calls.back().axes={axis};return 0;}

int32 __stdcall ZAux_Direct_GetAtype(ZMC_HANDLE handle, int iaxis, int *piValue) {
    CallGuard call("ZAux_Direct_GetAtype");
    if (!piValue) return -1;
    *piValue = (iaxis >= 4 && iaxis <= 5) ? 4 : 1;
    return 0;
}
