#include "FileDialog.h"

#include <QDir>
#include <QFileDialog>
#include <vector>

#include <io/File.h>

FileDialog::FileDialog(QWidget* parent)
    : mParent(parent)
    , mPath(QDir::homePath().toStdString())
{
}

std::vector<File> FileDialog::getOpenFileNames(
        std::string title, std::string options)
{
    QStringList list =
            QFileDialog::getOpenFileNames(mParent, QString::fromStdString(title),
                                  QString::fromStdString(mPath.getPath()), QString::fromStdString(options));

    std::vector<File> files;
    if (!list.empty())
    {
        for (QString s : list)
        {
            files.push_back(File(s.toStdString()));
        }
        mPath = File(files[0]);
        mPath = File(mPath.getRelativePath());
    }

    return files;
}
