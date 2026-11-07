#include "runtime/GCode5D.h"
#include <algorithm>
#include <cmath>
#include <set>
#include <limits>
namespace mls::runtime {
bool parseNcBlock(const std::string& line,NcState& state,NcBlock& block,std::string& error) {
    block={};std::vector<Word> words;
    if(!lex(line,words,error))return false;
    if(words.empty())return true;
    NcState next=state;NcBlock b;
    std::map<char,Word> params;std::set<int> groups;
    int motion=-1,mcode=-1,coordinate=-1;
    auto fail=[&](const std::string& s){error=s;return false;};
    for(const auto& w:words){
        if(w.letter=='G'||w.letter=='M'){
            if(w.text || w.number<0 || w.number>10000)return fail("Unsupported invalid G/M code");
            const bool g434 = w.letter=='G' && std::abs(w.number-43.4)<1e-9;
            if(!g434 && std::floor(w.number)!=w.number)return fail("Unsupported non-integral G/M code");
            const int code=int(std::floor(w.number));
            if(w.letter=='M'){
                if(mcode>=0 || !groups.empty())return fail("Only one M code; mixing G and M in a block is rejected");
                if(code!=0&&code!=2&&code!=30&&code!=50&&code!=51&&code!=60&&code!=61&&code!=62&&code!=63)return fail("Unsupported M"+std::to_string(code));
                mcode=code;continue;
            }
            if(mcode>=0)return fail("Mixed G/M block is not supported");
            int group=0;
            if(g434){group=6;next.rtcp=true;}
            else switch(code){
            case 0:case 1:case 2:case 3:case 4:case 5:case 92:group=1;motion=code;break;
            case 90:case 91:group=2;coordinate=code;next.absolute=code==90;break;
            case 17:case 18:case 19:group=3;next.plane=code==17?Plane::XY:(code==18?Plane::ZX:Plane::YZ);break;
            case 21:group=4;break;
            case 93:case 94:group=5;next.feedMode=code==93?FeedMode::InverseMinute:FeedMode::PerMinute;break;
            case 49:group=6;next.rtcp=false;break;
            default:return fail("Unsupported G"+std::to_string(code)+"; no silent RTCP/work-offset substitution");
            }
            if(!groups.insert(group).second)return fail("Conflicting/duplicate G modal group");
        }else{
            if(!params.emplace(w.letter,w).second)return fail("Duplicate word "+std::string(1,w.letter));
        }
    }
    b.absolute=next.absolute;b.feedMode=next.feedMode;b.plane=next.plane;b.rtcp=next.rtcp;
    b.arc.plane=next.plane;
    for(int j=0;j<5;++j){const char letter="XYZCB"[j];auto it=params.find(letter);if(it!=params.end()){
        if(it->second.text)return fail("Axis word must be numeric");
        b.words[j]=it->second.number;b.axisMask|=std::uint8_t(1u<<j);
    }}
    auto f=params.find('F');if(f!=params.end()){
        if(f->second.text || f->second.number<=0)return fail("F must be finite and positive");
        b.feed=f->second.number;
    }
    auto n=params.find('N');if(n!=params.end()){
        if(n->second.text || n->second.number<0 || n->second.number>1000000000 || std::floor(n->second.number)!=n->second.number)return fail("Invalid line number N");
        b.lineNumber=int(n->second.number);
    }
    if(mcode>=0){b.kind=NcBlock::Kind::MCode;b.mcode=mcode;}
    else{
        // G90 X.. / modal axis-only blocks inherit previous G0..G3, never invent one.
        if(motion<0 && (b.axisMask || params.count('I')||params.count('J')||params.count('K')||params.count('R'))){
            if(next.modalMotion<0)return fail("Axis/arc words before G0/G1/G2/G3/G5");
            motion=next.modalMotion;
        }
        if(motion==4){b.kind=NcBlock::Kind::Dwell;}
        else if(motion==92){b.kind=NcBlock::Kind::SetWorkPosition;if(!b.axisMask)return fail("G92 requires at least one axis word");}
        else if(motion>=0){b.kind=NcBlock::Kind::Motion;b.motion=motion;b.capture=(motion!=0);if(motion<=3 || motion==5)next.modalMotion=motion;}
        else b.kind=NcBlock::Kind::Modal;
    }
    std::string allowed="NL";
    switch(b.kind){
    case NcBlock::Kind::Motion: allowed+="XYZCBFS";if(motion==2||motion==3)allowed+="IJKRPQ";break;
    case NcBlock::Kind::SetWorkPosition:allowed+="XYZCB";break;
    case NcBlock::Kind::Dwell:allowed+="P";break;
    case NcBlock::Kind::Modal:allowed+="F";break;
    case NcBlock::Kind::MCode:if(mcode==62)allowed+="PQRT";break;
    default:break;
    }
    for(const auto& kv:params){const auto& w=kv.second;
        if(allowed.find(w.letter)==std::string::npos)return fail("Word "+std::string(1,w.letter)+" is not valid for this block");
        if(w.text){
            if(w.letter!='L' && !(mcode==62 && std::string("PQRT").find(w.letter)!=std::string::npos))return fail("Unexpected string word");
            b.strings[w.letter]=w.string;
        }else if(w.letter=='L' || (mcode==62 && std::string("PQRT").find(w.letter)!=std::string::npos))return fail("Text parameters require quoted strings");
    }
    if(b.kind==NcBlock::Kind::Dwell){
        auto it=params.find('P');if(it==params.end()||it->second.text||it->second.number<0||it->second.number>86400000||std::floor(it->second.number)!=it->second.number)return fail("G04 P is integer milliseconds in this project");
        b.dwellMs=int(it->second.number);
    }
    if(b.kind==NcBlock::Kind::Motion){
        auto it=params.find('S');if(it!=params.end()){
            if(it->second.text || (it->second.number!=0 && it->second.number!=1))return fail("S0/S1 is the capture flag, not a spindle command");
            b.capture=it->second.number==1;
        }
        if(motion<=1 && !b.axisMask)return fail("Linear block has no axis words");
        if(next.feedMode==FeedMode::InverseMinute && motion!=0 && !b.feed)return fail("G93 requires F on every motion block");
        if(motion==2||motion==3){
            b.arc.clockwise=motion==2;
            bool ijk=params.count('I')||params.count('J')||params.count('K');
            bool pq=params.count('P')||params.count('Q');bool r=params.count('R');
            if(int(ijk)+int(pq)+int(r)!=1)return fail("Arc requires exactly one definition: IJK, R, or XY P/Q");
            if(ijk){
                b.arcDefinition=NcBlock::ArcDefinition::Center;
                for(int j=0;j<3;++j){auto p=params.find("IJK"[j]);if(p!=params.end())b.arc.centerOffset[j]=p->second.number;}
                int normal=next.plane==Plane::XY?2:(next.plane==Plane::ZX?1:0);
                if(params.count("IJK"[normal]) && b.arc.centerOffset[normal]!=0)return fail("Out-of-plane center offset is not allowed");
            }else if(r){b.arcDefinition=NcBlock::ArcDefinition::Radius;b.arc.useRadius=true;b.arc.radius=params['R'].number;if(b.arc.radius==0)return fail("R must be nonzero");}
            else{
                if(next.plane!=Plane::XY || !params.count('P') || !params.count('Q'))return fail("Legacy three-point arc requires XY and both P,Q");
                b.arcDefinition=NcBlock::ArcDefinition::ThreePoint;b.mid[0]=params['P'].number;b.mid[1]=params['Q'].number;
            }
        }
    }
    (void)coordinate;
    state=next;block=std::move(b);return true;
}
}
