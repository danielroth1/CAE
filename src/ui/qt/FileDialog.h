#ifndef FILEDIALOG_H
#define FILEDIALOG_H

#include <QDir>
#include <QObject>
#include <QString>

#include <io/File.h>

// A utility class to simplfy the usage of QFileDialog. Provides methods
// to start file dialogs for selecting files and directories. Returns those
// files in the form of File objects.
class FileDialog : public QObject
{
    Q_OBJECT
public:
    FileDialog(QWidget* parent,
               const std::string& path = QDir::homePath().toStdString());

    // Opens a file dialog to select files.
    // If the dialog is closed, an empty vector is returned.
    // \param title - title of the file dialog, e.g. "Open File"
    // \param options - supported file formats, e.g. for images: "*.png *.jpg"
    std::vector<File> getOpenFileNames(std::string title, std::string options);

    // Starts a dialog with the given title so select a file for saving.
    // If the dialog is closed, a File with an empty path string "" is retured.
    // \param title. title of the file dialog, e.g. "Save File"
    // \param options - supported file formats, e.g. for images "*.png *.jpg"
    File getSaveFileName(std::string title, std::string options);

    // Starts a dialog to select a directory.
    // If the dialog is closed, a File with an empty path string "" is returned.
    // \param title - the title of the dialog, e.g. "Save Files Directory"
    File getDirectory(std::string title);

private:
    QWidget* mParent;
    File mPath;
};

#endif // FILEDIALOG_H
