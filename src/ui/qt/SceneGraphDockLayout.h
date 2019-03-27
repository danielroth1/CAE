#ifndef SCENEGRAPHDOCKLAYOUT_H
#define SCENEGRAPHDOCKLAYOUT_H

#include <QWidget>

class SceneGraphDockLayout : public QWidget
{
    Q_OBJECT
public:
    explicit SceneGraphDockLayout(QWidget *parent = nullptr);

    // QWidget interface
public:
    virtual QSize sizeHint() const;

signals:

public slots:

};

#endif // SCENEGRAPHDOCKLAYOUT_H
