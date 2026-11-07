#pragma once
#include "core/DataTypes.h"
#include "QtShim.h"
// TEST ONLY. No LLT SDK calls or sensor emulation.
class LaserSensor : public QObject {
public:
    struct ProfileCallbackDiagnostics {
        unsigned long long rawCallbacks=0, convertedProfiles=0, conversionFailures=0;
        unsigned long long ingressDrops=0, rawRingDepth=0, rawRingHighWater=0, rawRingCapacity=128, lastCallbackOrdinal=0;
        long long maxCallbackServiceUs=0, maxDecoderQueueUs=0, maxDecodeServiceUs=0;
        int lastConvertResult=0; unsigned int lastCallbackBytes=0, lastResolution=0;
    };
    struct HardwareTriggerDiagnostics { bool readbackValid=true; unsigned int triggerRequested=0, triggerReadback=0, digitalIoRequested=0, digitalIoReadback=0; };
    bool isConnected() const { return true; }
    static int getMaxFrequencyForIndex(int index) {
        switch (index) {
        case 0: return 144; case 2: return 192; case 7: return 552;
        case 32: return 241; case 64: return 362; case 96: return 552; case 122: return 2000;
        default: return 2000;
        }
    }
    bool configure(SensorTriggerMode, const SensorParams&, bool, HardwareTriggerType, QString*) { return true; }
    bool setMeasuringFieldIndex(int,bool,QString*){return true;}
    bool configureCmmOutput(QString*){return true;}
    bool startCmmGated(int,QString*){return true;}
    bool stopCmmGated(QString*){return true;}
    bool startContinuous(QString*) { return true; }
    void stopContinuous() {}
    ProfileCallbackDiagnostics profileCallbackDiagnostics() const {
        ProfileCallbackDiagnostics d; d.rawCallbacks=8; d.convertedProfiles=8; d.lastCallbackOrdinal=8; d.rawRingCapacity=128; d.lastConvertResult=6144; d.lastCallbackBytes=81920; d.lastResolution=1280; return d;
    }
    HardwareTriggerDiagnostics hardwareTriggerDiagnostics() const { return {}; }
    void resetProfileCallbackDiagnostics() {}
    void setProfileEmissionSuppressed(bool) {}
    void profileReceived(const LaserProfile&) {}
    void profileIngressFault(const QString&) {}
};
