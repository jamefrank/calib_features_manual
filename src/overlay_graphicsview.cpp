# include "overlay_graphicsview.h"

#include "overlay_graphicsview.h"
#include <QPainter>

OverlayGraphicsView::OverlayGraphicsView(QWidget *parent)
    : QGraphicsView(parent)
{
    // 1. 创建 scene
    setScene(new QGraphicsScene(this));
    setFocusPolicy(Qt::NoFocus);
    setDragMode(QGraphicsView::NoDrag);
    scene()->setSceneRect(0, 0, 100, 100); // 初始大小，后面会更新

    // 2. 关键：关闭 viewport 背景填充
    viewport()->setAutoFillBackground(false);

    // 3. 设置 viewport 的调色板：必须设置 Base 为透明！
    QPalette p = viewport()->palette();
    p.setColor(QPalette::Window, Qt::transparent);
    p.setColor(QPalette::Background, Qt::transparent);
    p.setColor(QPalette::Base, Qt::transparent);        // ✅ 最关键的一行！
    viewport()->setPalette(p);

    // 4. 设置 view 属性
    setAttribute(Qt::WA_TranslucentBackground);
    setAttribute(Qt::WA_NoSystemBackground);
    setStyleSheet("background:transparent;"); // 可选：额外保险

    // 5. 确保没有多余绘制
    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
}

void OverlayGraphicsView::paintEvent(QPaintEvent* event) {
    
}
