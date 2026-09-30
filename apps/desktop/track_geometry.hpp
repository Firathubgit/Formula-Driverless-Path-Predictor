#pragma once
#include <QQuick3DGeometry>
#include <QPointer>
#include <vector>
class Bridge;
class TrackGeometry : public QQuick3DGeometry {
    Q_OBJECT
    Q_PROPERTY(int kind READ kind WRITE setKind NOTIFY kindChanged)
    Q_PROPERTY(QObject* bridge READ bridge WRITE setBridge NOTIFY bridgeChanged)
public:
    explicit TrackGeometry(QQuick3DObject* parent=nullptr):QQuick3DGeometry(parent){}
    int kind() const {return kind_;}
    void setKind(int kind);
    QObject* bridge() const;
    void setBridge(QObject* bridge);
signals:
    void kindChanged();
    void bridgeChanged();
private:
    void rebuild();
    int kind_{};
    QPointer<Bridge> bridge_;
    std::vector<std::size_t> lattice_layers_;  // the lattice layers last drawn
    const void* lattice_drawn_{};
};
