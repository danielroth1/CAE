#include "FileDialog.h"

#include <QDir>
#include <QFileDialog>
#include <vector>

#include <io/File.h>

FileDialog::FileDialog(QWidget* parent, const std::string& path)
    : mParent(parent)
    , mPath(path)
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

File FileDialog::getSaveFileName(std::string title, std::string options)
{
    QString qFile =
            QFileDialog::getSaveFileName(
                mParent,
                QString::fromStdString(title),
                QString::fromStdString(mPath.getPath()),
                QString::fromStdString(options));

    return File(qFile.toStdString());
}

File FileDialog::getDirectory(std::string title)
{
    QString qDir =
            QFileDialog::getExistingDirectory(
                mParent,
                QString::fromStdString(title),
                QString::fromStdString(mPath.getPath()));

    return File(qDir.toStdString());
}
