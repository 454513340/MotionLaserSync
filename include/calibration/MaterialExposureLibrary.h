#pragma once

#include <QVector>
#include <QString>

namespace mls::calibration {

struct MaterialExposureProfile {
    QString key;
    QString material;
    QString materialFileSafe;
    QString lightingCode;
    QString lightingDisplay;
    double temperatureC = 25.0;

    QString sensorTypeName;
    QString sensorSerialNumber;
    int measuringFieldIndex = 0;
    int resolution = 0;
    int laserPower = 2;
    int reflectionSelect = 3;
    double thresholdPercent = 12.5;

    // Calibration provenance. Pose is not used to recompute Idle, but is part
    // of the material-profile identity so distinct stand-off/orientation tests
    // are not silently overwritten.
    bool sourcePoseValid = false;
    double sourceX = 0.0;
    double sourceY = 0.0;
    double sourceZ = 0.0;
    double sourceC = 0.0;
    double sourceB = 0.0;

    int recommendedExposure10us = 0;
    int referenceIdle10us = -1;
    int referenceProfileRateHz = 0;
    int saturationOnsetExposure10us = 0;
    int safeCandidateMinExposure10us = 0;
    int safeCandidateMaxExposure10us = 0;
    double inferredSaturationCeiling = 0.0;
    QString confidence;
    QString algorithmVersion;
    QString sourceSessionDirectory;
    QString updatedUtc;
};

class MaterialExposureLibrary {
public:
    MaterialExposureLibrary() = default;

    bool load(QString* error = nullptr);
    bool save(QString* error = nullptr) const;
    bool upsert(MaterialExposureProfile profile, QString* error = nullptr);

    const QVector<MaterialExposureProfile>& entries() const { return m_entries; }
    bool findByKey(const QString& key, MaterialExposureProfile& profile) const;
    QString storagePath() const;

    static QString makeKey(const MaterialExposureProfile& profile);

private:
    bool loadFile(const QString& path, QVector<MaterialExposureProfile>& target, QString* error) const;
    static void mergeEntry(QVector<MaterialExposureProfile>& target, MaterialExposureProfile profile);

    QVector<MaterialExposureProfile> m_entries;
};

} // namespace mls::calibration
