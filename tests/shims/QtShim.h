#pragma once
// TEST ONLY. Minimal C++ stand-ins for compiling/exercising the production core
// without installed Qt. NOT a Qt implementation: no moc, signal delivery, Qt
// implicit sharing or real event loop is tested by this harness.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>
using qint64 = long long;
using quint32 = std::uint32_t;
using quint64 = unsigned long long;
using qulonglong = unsigned long long;
using qsizetype = std::ptrdiff_t;
#define Q_OBJECT
#define Q_UNUSED(x) (void)(x);
#define Q_DECLARE_METATYPE(...)
#define signals public
#define slots
#define emit
namespace Qt { enum ConnectionType { AutoConnection, QueuedConnection, DirectConnection }; }
class QByteArray {
    std::vector<char> data_;
public:
    explicit QByteArray(const std::string& s) : data_(s.begin(), s.end()) {}
    void append(char c) { data_.push_back(c); }
    char* data() { return data_.data(); }
};
class QString {
    std::string value_;
public:
    QString() = default;
    QString(const char* s) : value_(s ? s : "") {}
    QString(std::string s) : value_(std::move(s)) {}
    bool isEmpty() const { return value_.empty(); }
    void clear() { value_.clear(); }
    QString toUpper() const { return *this; }
    QByteArray toUtf8() const { return QByteArray(value_); }
    static QString fromUtf8(const char* s) { return QString(s); }
    static QString fromLatin1(const char* s) { return QString(s); }
    static QString fromStdString(const std::string& s) { return QString(s.c_str()); }
    const std::string& str() const { return value_; }
    template<class T, class... Format> QString arg(const T& value, Format...) const {
        std::ostringstream stream;
        stream << value;
        std::string text = value_;
        const auto pos = text.find('%');
        if (pos != std::string::npos) {
            auto end = pos + 1;
            while (end < text.size() && text[end] >= '0' && text[end] <= '9') ++end;
            text.replace(pos, end - pos, stream.str());
        }
        return QString(text);
    }
    QString& operator+=(const QString& other) { value_ += other.value_; return *this; }
    friend QString operator+(QString a, const QString& b) { a += b; return a; }
    friend std::ostream& operator<<(std::ostream& os, const QString& s) { return os << s.value_; }
};
#define QStringLiteral(s) QString(s)
inline char QLatin1Char(char c) { return c; }
template<class T> class QVector : public std::vector<T> {
public:
    using std::vector<T>::vector;
    qsizetype size() const { return static_cast<qsizetype>(std::vector<T>::size()); }
    bool isEmpty() const { return this->empty(); }
    void append(const T& t) { this->push_back(t); }
    void append(const QVector& items) { this->insert(this->end(), items.begin(), items.end()); }
    const T& first() const { return this->front(); }
    const T& last() const { return this->back(); }
};
class QStringList : public QVector<QString> {
public:
    QStringList& operator<<(const QString& s) { append(s); return *this; }
    QString join(const QString& delimiter) const {
        QString out;
        for (std::size_t i = 0; i < static_cast<std::size_t>(size()); ++i) {
            if (i) out += delimiter;
            out += (*this)[i];
        }
        return out;
    }
};
class QVector3D {};
class QMatrix4x4 {};
class QMutex : public std::mutex {};
template<class Mutex> class QMutexLocker {
    Mutex* mutex_;
    bool owns_ = true;
public:
    explicit QMutexLocker(Mutex* m) : mutex_(m) { m->lock(); }
    ~QMutexLocker() { if (owns_) mutex_->unlock(); }
    void unlock() { if (owns_) { mutex_->unlock(); owns_ = false; } }
};
class QThread;
struct QMetaObject {
    struct Connection {
        bool valid = false;
        explicit operator bool() const { return valid; }
    };
    template<class F> static bool invokeMethod(class QObject*, F&&, Qt::ConnectionType) { return true; }
};
inline QThread* shimCurrentThread() {
    static thread_local unsigned char identity;
    return reinterpret_cast<QThread*>(&identity); // identity only, never dereferenced
}
class QObject {
    QThread* owner_ = shimCurrentThread();
public:
    explicit QObject(QObject* = nullptr) {}
    virtual ~QObject() = default;
    QThread* thread() const { return owner_; }
    static QString tr(const char* s) { return QString(s); }
    template<class... Args> static QMetaObject::Connection connect(Args&&...) { return {true}; } // no signal/event semantics
    static bool disconnect(const QMetaObject::Connection&) { return true; }
};
class QThread : public QObject {
    std::thread thread_;
    std::mutex finishMutex_;
    std::condition_variable finishCv_;
    bool finished_ = true;
protected:
    virtual void run() {}
public:
    using QObject::QObject;
    enum Priority { HighPriority, HighestPriority, TimeCriticalPriority };
    ~QThread() override { if (thread_.joinable()) thread_.join(); }
    static QThread* currentThread() { return shimCurrentThread(); }
    static void msleep(unsigned long t) { std::this_thread::sleep_for(std::chrono::milliseconds(t)); }
    static void usleep(unsigned long t) { std::this_thread::sleep_for(std::chrono::microseconds(t)); }
    void setPriority(Priority) {}
    void start() {
        finished_ = false;
        thread_ = std::thread([this] {
            run();
            { std::lock_guard<std::mutex> lock(finishMutex_); finished_ = true; }
            finishCv_.notify_all();
        });
    }
    bool wait(unsigned long ms = 0) {
        std::unique_lock<std::mutex> lock(finishMutex_);
        if (ms == 0) finishCv_.wait(lock, [this] { return finished_; });
        else if (!finishCv_.wait_for(lock, std::chrono::milliseconds(ms), [this] { return finished_; })) return false;
        lock.unlock();
        if (thread_.joinable()) thread_.join();
        return true;
    }
};
class QElapsedTimer {
    std::chrono::steady_clock::time_point start_ = std::chrono::steady_clock::now();
public:
    void start() { start_ = std::chrono::steady_clock::now(); }
    qint64 nsecsElapsed() const {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now()-start_).count();
    }
    qint64 elapsed() const { return nsecsElapsed() / 1000000; }
};
class QTimer : public QObject {
public:
    template<class C,class F> static void singleShot(int,C*,F&&) {} // event loop is deliberately NOT simulated

    void setInterval(int) {}
    void start() {}
    void stop() {}
    void timeout() {}
};
struct QEvent { enum { MetaCall = 1 }; };
struct QEventLoop { enum { ExcludeUserInputEvents = 1 }; };
struct QCoreApplication {
    static void processEvents(int = 0) {}
    static void sendPostedEvents(QObject*, int) {}
};
struct ShimDebug {
    ShimDebug& noquote() { return *this; }
    template<class T> ShimDebug& operator<<(const T&) { return *this; }
};
inline ShimDebug qInfo() { return {}; }
inline ShimDebug qDebug() { return {}; }
inline ShimDebug qWarning() { return {}; }
template<class A, class B> auto qMin(A a, B b) { return std::min<std::common_type_t<A,B>>(a,b); }
template<class A, class B> auto qMax(A a, B b) { return std::max<std::common_type_t<A,B>>(a,b); }
