#ifndef FILEDIALOG_H
#define FILEDIALOG_H

#include <QObject>
#include <QString>

#include <io/File.h>


class FileDialog : public QObject
{
    Q_OBJECT
public:
    FileDialog(QWidget* parent);

    // @param title
    // @param options - e.g. "Images (*.png *.xpm *.jpg)"
    std::vector<File> getOpenFileNames(std::string title, std::string options);

private:
    QWidget* mParent;
    File mPath;
};

#endif // FILEDIALOG_H
