#include "SceneGraphDockLayout.h"

SceneGraphDockLayout::SceneGraphDockLayout(QWidget *parent) : QWidget(parent)
{

}

QSize SceneGraphDockLayout::sizeHint() const
{
    return QSize(200, 524287);
}
