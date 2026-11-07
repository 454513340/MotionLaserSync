#include "calibration/MaterialExposureLibrary.h"

#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>
#include <QStandardPaths>
#include <QStringList>

#include <utility>

namespace mls::calibration {
namespace {

QJsonObject toJson(const MaterialExposureProfile& p)
{
    QJsonObject o;
    o[QStringLiteral("key")] = p.key;
    o[QStringLiteral("material")] = p.material;
    o[QStringLiteral("materialFileSafe")] = p.materialFileSafe;
    o[QStringLiteral("lightingCode")] = p.lightingCode;
    o[QStringLiteral("lightingDisplay")] = p.lightingDisplay;
    o[QStringLiteral("temperatureC")] = p.temperatureC;
    o[QStringLiteral("sensorTypeName")] = p.sensorTypeName;
    o[QStringLiteral("sensorSerialNumber")] = p.sensorSerialNumber;
    o[QStringLiteral("measuringFieldIndex")] = p.measuringFieldIndex;
    o[QStringLiteral("resolution")] = p.resolution;
    o[QStringLiteral("laserPower")] = p.laserPower;
    o[QStringLiteral("reflectionSelect")] = p.reflectionSelect;
    o[QStringLiteral("thresholdPercent")] = p.thresholdPercent;
    QJsonObject pose;
    pose[QStringLiteral("valid")] = p.sourcePoseValid;
    pose[QStringLiteral("x")] = p.sourceX;
    pose[QStringLiteral("y")] = p.sourceY;
    pose[QStringLiteral("z")] = p.sourceZ;
    pose[QStringLiteral("c")] = p.sourceC;
    pose[QStringLiteral("b")] = p.sourceB;
    o[QStringLiteral("sourcePoseXYZBC")] = pose;
    o[QStringLiteral("recommendedExposure_10us")] = p.recommendedExposure10us;
    o[QStringLiteral("referenceIdle_10us")] = p.referenceIdle10us;
    o[QStringLiteral("referenceProfileRateHz")] = p.referenceProfileRateHz;
    o[QStringLiteral("saturationOnset_10us")] = p.saturationOnsetExposure10us;
    QJsonArray safeBand;
    safeBand.append(p.safeCandidateMinExposure10us);
    safeBand.append(p.safeCandidateMaxExposure10us);
    o[QStringLiteral("safeCandidateBand_10us")] = safeBand;
    o[QStringLiteral("inferredSaturationCeiling")] = p.inferredSaturationCeiling;
    o[QStringLiteral("confidence")] = p.confidence;
    o[QStringLiteral("algorithmVersion")] = p.algorithmVersion;
    o[QStringLiteral("sourceSessionDirectory")] = p.sourceSessionDirectory;
    o[QStringLiteral("updatedUtc")] = p.updatedUtc;
    return o;
}

MaterialExposureProfile fromJson(const QJsonObject& o)
{
    MaterialExposureProfile p;
    p.key = o.value(QStringLiteral("key")).toString();
    p.material = o.value(QStringLiteral("material")).toString();
    p.materialFileSafe = o.value(QStringLiteral("materialFileSafe")).toString();
    p.lightingCode = o.value(QStringLiteral("lightingCode")).toString();
    p.lightingDisplay = o.value(QStringLiteral("lightingDisplay")).toString();
    p.temperatureC = o.value(QStringLiteral("temperatureC")).toDouble(25.0);
    p.sensorTypeName = o.value(QStringLiteral("sensorTypeName")).toString();
    p.sensorSerialNumber = o.value(QStringLiteral("sensorSerialNumber")).toString();
    p.measuringFieldIndex = o.value(QStringLiteral("measuringFieldIndex")).toInt(0);
    p.resolution = o.value(QStringLiteral("resolution")).toInt(0);
    p.laserPower = o.value(QStringLiteral("laserPower")).toInt(2);
    p.reflectionSelect = o.value(QStringLiteral("reflectionSelect")).toInt(3);
    p.thresholdPercent = o.value(QStringLiteral("thresholdPercent")).toDouble(12.5);
    const QJsonObject pose = o.value(QStringLiteral("sourcePoseXYZBC")).toObject();
    p.sourcePoseValid = pose.value(QStringLiteral("valid")).toBool(false);
    p.sourceX = pose.value(QStringLiteral("x")).toDouble(0.0);
    p.sourceY = pose.value(QStringLiteral("y")).toDouble(0.0);
    p.sourceZ = pose.value(QStringLiteral("z")).toDouble(0.0);
    p.sourceC = pose.value(QStringLiteral("c")).toDouble(0.0);
    p.sourceB = pose.value(QStringLiteral("b")).toDouble(0.0);
    p.recommendedExposure10us = o.value(QStringLiteral("recommendedExposure_10us")).toInt(0);
    p.referenceIdle10us = o.value(QStringLiteral("referenceIdle_10us")).toInt(-1);
    p.referenceProfileRateHz = o.value(QStringLiteral("referenceProfileRateHz")).toInt(0);
    p.saturationOnsetExposure10us = o.value(QStringLiteral("saturationOnset_10us")).toInt(0);
    const QJsonArray safeBand = o.value(QStringLiteral("safeCandidateBand_10us")).toArray();
    if (safeBand.size() >= 2) {
        p.safeCandidateMinExposure10us = safeBand.at(0).toInt(0);
        p.safeCandidateMaxExposure10us = safeBand.at(1).toInt(0);
    }
    p.inferredSaturationCeiling = o.value(QStringLiteral("inferredSaturationCeiling")).toDouble(0.0);
    p.confidence = o.value(QStringLiteral("confidence")).toString();
    p.algorithmVersion = o.value(QStringLiteral("algorithmVersion")).toString();
    p.sourceSessionDirectory = o.value(QStringLiteral("sourceSessionDirectory")).toString();
    p.updatedUtc = o.value(QStringLiteral("updatedUtc")).toString();
    // The key is canonical and derived from the calibration context. Rebuild it
    // on every load so pre-release/older key formats migrate automatically.
    p.key = MaterialExposureLibrary::makeKey(p);
    return p;
}

} // namespace

QString MaterialExposureLibrary::makeKey(const MaterialExposureProfile& p)
{
    const QString material = !p.materialFileSafe.isEmpty() ? p.materialFileSafe : p.material;
    const QString sensor = !p.sensorSerialNumber.isEmpty() ? p.sensorSerialNumber : p.sensorTypeName;
    QString key = QStringLiteral("%1|%2|%3|mf%4|r%5|lp%6|refl%7|th%8|t%9|hz%10")
        .arg(material.trimmed().toLower())
        .arg(p.lightingCode.trimmed().toLower())
        .arg(sensor.trimmed().toLower())
        .arg(p.measuringFieldIndex)
        .arg(p.resolution)
        .arg(p.laserPower)
        .arg(p.reflectionSelect)
        .arg(p.thresholdPercent, 0, 'f', 2)
        .arg(p.temperatureC, 0, 'f', 1)
        .arg(p.referenceProfileRateHz);
    if (p.sourcePoseValid) {
        key += QStringLiteral("|pose_x%1_y%2_z%3_c%4_b%5")
            .arg(p.sourceX, 0, 'f', 1)
            .arg(p.sourceY, 0, 'f', 1)
            .arg(p.sourceZ, 0, 'f', 1)
            .arg(p.sourceC, 0, 'f', 1)
            .arg(p.sourceB, 0, 'f', 1);
    } else {
        key += QStringLiteral("|pose_na");
    }
    return key;
}

QString MaterialExposureLibrary::storagePath() const
{
    // V27.6.45: keep the material exposure library together with the rest of
    // the persistent calibration data.  On Windows AppDataLocation resolves to
    // %APPDATA% (Roaming), i.e.
    // C:/Users/<user>/AppData/Roaming/YourCompany/MotionLaserSync.
    QString base = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
#ifdef Q_OS_WIN
    if (base.isEmpty()) {
        base = QDir::home().filePath(QStringLiteral("AppData/Roaming/YourCompany/MotionLaserSync"));
    }
#else
    if (base.isEmpty()) base = QCoreApplication::applicationDirPath();
#endif
    return QDir(base).filePath(QStringLiteral("calibration/material_exposure_library.json"));
}

void MaterialExposureLibrary::mergeEntry(QVector<MaterialExposureProfile>& target,
                                         MaterialExposureProfile profile)
{
    profile.key = makeKey(profile);
    for (auto& current : target) {
        if (current.key == profile.key) {
            current = std::move(profile);
            return;
        }
    }
    target.push_back(std::move(profile));
}

bool MaterialExposureLibrary::loadFile(const QString& path,
                                       QVector<MaterialExposureProfile>& target,
                                       QString* error) const
{
    QFile file(path);
    if (!file.exists()) return true;
    if (!file.open(QIODevice::ReadOnly)) {
        if (error) *error = QStringLiteral("无法读取曝光材料库 %1: %2").arg(path, file.errorString());
        return false;
    }
    QJsonParseError parseError;
    const QJsonDocument doc = QJsonDocument::fromJson(file.readAll(), &parseError);
    if (parseError.error != QJsonParseError::NoError || !doc.isObject()) {
        if (error) *error = QStringLiteral("曝光材料库JSON无效 %1: %2").arg(path, parseError.errorString());
        return false;
    }
    const QJsonArray entries = doc.object().value(QStringLiteral("entries")).toArray();
    for (const auto& value : entries) {
        if (!value.isObject()) continue;
        MaterialExposureProfile p = fromJson(value.toObject());
        if (p.recommendedExposure10us <= 0) continue;
        mergeEntry(target, std::move(p));
    }
    return true;
}

bool MaterialExposureLibrary::load(QString* error)
{
    QVector<MaterialExposureProfile> loaded;

    // Bundled defaults are read first; user/session-derived results with the
    // same key override them. CMake copies config next to the executable.
    QStringList defaultCandidates;
    defaultCandidates << QDir(QCoreApplication::applicationDirPath())
                            .filePath(QStringLiteral("config/material_exposure_defaults.json"));
    defaultCandidates << QDir(QDir::currentPath())
                            .filePath(QStringLiteral("config/material_exposure_defaults.json"));
    for (const QString& path : defaultCandidates) {
        if (QFile::exists(path)) {
            QString localError;
            if (!loadFile(path, loaded, &localError) && error && error->isEmpty()) *error = localError;
            break;
        }
    }

    const QString currentPath = storagePath();
    QString userError;

    // V27.6.45 one-time compatibility migration.  V27.6.43/44 stored the user
    // library under AppConfigLocation (observed on Windows under AppData/Local).
    // If the new Roaming/calibration library does not exist yet, read the old
    // file once and persist the merged result to the new calibration directory.
    bool loadedLegacy = false;
    if (!QFile::exists(currentPath)) {
        QString legacyDir = QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation);
        if (!legacyDir.isEmpty()) {
            const QString legacyPath = QDir(legacyDir).filePath(QStringLiteral("material_exposure_library.json"));
            if (legacyPath != currentPath && QFile::exists(legacyPath)) {
                if (!loadFile(legacyPath, loaded, &userError)) {
                    if (error) *error = userError;
                    return false;
                }
                loadedLegacy = true;
            }
        }
    }

    if (!loadFile(currentPath, loaded, &userError)) {
        if (error) *error = userError;
        return false;
    }
    m_entries = std::move(loaded);

    if (loadedLegacy) {
        QString migrationError;
        if (!save(&migrationError)) {
            if (error) *error = QStringLiteral("旧材料曝光库已读取，但迁移到新标定目录失败: %1").arg(migrationError);
            return false;
        }
    }
    return true;
}

bool MaterialExposureLibrary::save(QString* error) const
{
    const QString path = storagePath();
    QDir parent(QFileInfo(path).absolutePath());
    if (!parent.exists() && !parent.mkpath(QStringLiteral("."))) {
        if (error) *error = QStringLiteral("无法创建曝光材料库目录: %1").arg(parent.absolutePath());
        return false;
    }

    QJsonObject root;
    root[QStringLiteral("schema")] = QStringLiteral("MotionLaserSync.MaterialExposureLibrary.v1");
    root[QStringLiteral("applicationVersion")] = QStringLiteral("V27.6.47");
    root[QStringLiteral("updatedUtc")] = QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs);
    QJsonArray entries;
    for (const auto& p : m_entries) entries.append(toJson(p));
    root[QStringLiteral("entries")] = entries;

    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly)) {
        if (error) *error = QStringLiteral("无法写入曝光材料库 %1: %2").arg(path, file.errorString());
        return false;
    }
    file.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    if (!file.commit()) {
        if (error) *error = QStringLiteral("曝光材料库原子提交失败: %1").arg(file.errorString());
        return false;
    }
    return true;
}

bool MaterialExposureLibrary::upsert(MaterialExposureProfile profile, QString* error)
{
    if (profile.recommendedExposure10us <= 0) {
        if (error) *error = QStringLiteral("推荐曝光必须大于0");
        return false;
    }
    if (profile.updatedUtc.isEmpty())
        profile.updatedUtc = QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs);
    profile.key = makeKey(profile);
    mergeEntry(m_entries, std::move(profile));
    return save(error);
}

bool MaterialExposureLibrary::findByKey(const QString& key,
                                        MaterialExposureProfile& profile) const
{
    for (const auto& p : m_entries) {
        if (p.key == key) {
            profile = p;
            return true;
        }
    }
    return false;
}

} // namespace mls::calibration
