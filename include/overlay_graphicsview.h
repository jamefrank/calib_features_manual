#pragma once

#include <QGraphicsView>
#include <qevent.h>
#include <qgraphicsview.h>
#include <qobjectdefs.h>
#include <QGraphicsItem>


class OverlayGraphicsView : public QGraphicsView{
    Q_OBJECT
public:
    explicit OverlayGraphicsView(QWidget *parent = nullptr);

protected:
//     void mousePressEvent(QMouseEvent* event) override;
//     void mouseMoveEvent(QMouseEvent* event) override;
//     void mouseReleaseEvent(QMouseEvent* event) override;
//     void wheelEvent(QWheelEvent* event) override;
//     void keyPressEvent(QKeyEvent* event) override;
//     void keyReleaseEvent(QKeyEvent* event) override;
    void paintEvent(QPaintEvent* event);

};