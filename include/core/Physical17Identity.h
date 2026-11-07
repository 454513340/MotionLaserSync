#pragma once
// Deterministic identity of the exact 17-parameter numerical model.
// An identity/fingerprint is NOT a calibration acceptance certificate.
#include <QByteArray>
#include <QCryptographicHash>
#include <QDataStream>
#include <QIODevice>
#include <QString>
#include <array>
#include <cmath>

namespace mls::identity {
inline QString physical17Sha256(const std::array<double,17>& values) {
    QByteArray payload;
    QDataStream out(&payload, QIODevice::WriteOnly);
    out.setByteOrder(QDataStream::BigEndian);
    out.setFloatingPointPrecision(QDataStream::DoublePrecision);
    static constexpr char tag[] = "MLS-PHYSICAL17-IEEE754-BE-V1";
    out.writeRawData(tag, sizeof(tag)-1);
    for (double value : values) {
        if (!std::isfinite(value)) return {};
        // Normalize signed zero so mathematically identical imported settings
        // have one immutable content identity.
        out << (value == 0.0 ? 0.0 : value);
    }
    if (out.status() != QDataStream::Ok) return {};
    return QString::fromLatin1(
        QCryptographicHash::hash(payload, QCryptographicHash::Sha256).toHex());
}
inline QString candidateCalibrationId(const std::array<double,17>& parameters) {
    const QString digest = physical17Sha256(parameters);
    return digest.isEmpty() ? QString() : QStringLiteral("CAL17_IMPORTED_SHA256_") + digest;
}
inline QString candidatePoseId(const QByteArray& cadSha256,
                               const std::array<double,16>& cadToMachine) {
    if (cadSha256.size()!=64) return {};
    QByteArray payload;
    QDataStream out(&payload, QIODevice::WriteOnly);
    out.setByteOrder(QDataStream::BigEndian);
    out.setFloatingPointPrecision(QDataStream::DoublePrecision);
    static constexpr char tag[] = "MLS-PART-POSE-CANDIDATE-V1";
    out.writeRawData(tag, sizeof(tag)-1);
    out.writeRawData(cadSha256.constData(), cadSha256.size());
    for (double value : cadToMachine) {
        if (!std::isfinite(value)) return {};
        out << (value == 0.0 ? 0.0 : value);
    }
    if(out.status()!=QDataStream::Ok)return {};
    return QStringLiteral("POSE_CANDIDATE_SHA256_")+
        QString::fromLatin1(QCryptographicHash::hash(payload,QCryptographicHash::Sha256).toHex());
}
}
