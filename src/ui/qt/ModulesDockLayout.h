#ifndef MODULESDOCKLAYOUT_H
#define MODULESDOCKLAYOUT_H

#include <QWidget>



class ModulesDockLayout : public QWidget
{
    Q_OBJECT
public:
    explicit ModulesDockLayout(QWidget *parent = nullptr);

    // QWidget interface
public:
    virtual QSize sizeHint() const;

signals:

public slots:


};

#endif // MODULESDOCKLAYOUT_H
