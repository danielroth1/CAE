#ifndef QTDEFERREDELEMENTSELECTIONWIDGET_CPP
#define QTDEFERREDELEMENTSELECTIONWIDGET_CPP

#include "QtDeferredElementSelectionWidget.h"

#include <QHBoxLayout>

#include <iostream>

template<class T>
QtDeferredElementSelectionWidget<T>::QtDeferredElementSelectionWidget(
        QWidget* parent,
        bool addChangeSelectionButton)
    : QWidget(parent)
{

    QHBoxLayout* layout = new QHBoxLayout();
    setLayout(layout);
    layout->setContentsMargins(0, 0, 0, 0);

    float buttonHeight = 32;

    mComboBox = new QComboBox();
    layout->addWidget(mComboBox);

    mButtonAddSelection = new QPushButton("M");
    mButtonAddSelection->setToolTip("Press to set the selection mode (waits for global selection). Release to select the current globally selected element.");
    mButtonAddSelection->setCheckable(true);
    mButtonAddSelection->setMinimumWidth(buttonHeight);
    mButtonAddSelection->setMaximumWidth(buttonHeight);
    layout->addWidget(mButtonAddSelection);

    if (addChangeSelectionButton)
    {
        mButtonChangeSelection = new QPushButton("S");
        mButtonChangeSelection->setToolTip("Set the global selection.");
        mButtonChangeSelection->setMinimumWidth(buttonHeight);
        mButtonChangeSelection->setMaximumWidth(buttonHeight);
        layout->addWidget(mButtonChangeSelection);
    }
    else
    {
        mButtonChangeSelection = nullptr;
    }
}

template<class T>
QtDeferredElementSelectionWidget<T>::~QtDeferredElementSelectionWidget()
{

}

template<class T>
void QtDeferredElementSelectionWidget<T>::selectElements(
        const std::vector<T>& selectedElements,
        const std::vector<std::string>& names)
{
    if (selectedElements.size() != names.size())
    {
        std::cout << "Different number of selected elements and names.\n";
        return;
    }

    mSelectedElements = selectedElements;

    mComboBox->clear();
    for (size_t i = 0; i < names.size(); ++i)
    {
        mComboBox->addItem(QString::fromStdString(names[i]));
    }
}

template<class T>
void QtDeferredElementSelectionWidget<T>::selectElementsIfButtonPressed(
        const std::vector<T>& selectedElements,
        const std::vector<std::string>& names)
{
    if (mButtonAddSelection->isChecked())
    {
        selectElements(selectedElements, names);
        mButtonAddSelection->blockSignals(true);
        mButtonAddSelection->setChecked(false);
        mButtonAddSelection->blockSignals(false);
    }
}

template<class T>
void QtDeferredElementSelectionWidget<T>::updateNames(const std::vector<std::string>& names)
{
    if (names.size() != mSelectedElements.size())
    {
        std::cout << "Size missmatch. Can not update names.\n";
        return;
    }

    mComboBox->clear();
    for (size_t i = 0; i < names.size(); ++i)
    {
        mComboBox->addItem(QString::fromStdString(names[i]));
    }
}

template<class T>
void QtDeferredElementSelectionWidget<T>::updateName(size_t index, const std::string& name)
{
    mComboBox->setItemText(index, QString::fromStdString(name));
}

template<class T>
size_t QtDeferredElementSelectionWidget<T>::getIndex(T element)
{
    for (size_t i = 0; i < mSelectedElements.size(); ++i)
    {
        if (mSelectedElements[i] == element)
            return i;
    }
    return 0;
}

template<class T>
T QtDeferredElementSelectionWidget<T>::getSelectedElement() const
{
    if (static_cast<size_t>(mComboBox->currentIndex()) >= mSelectedElements.size())
        return nullptr;

    return mSelectedElements[mComboBox->currentIndex()];
}

template<class T>
std::vector<T> QtDeferredElementSelectionWidget<T>::getSelectedElements() const
{
    return mSelectedElements;
}

template<class T>
QPushButton* QtDeferredElementSelectionWidget<T>::getButtonSetSelection()
{
    return mButtonAddSelection;
}

template<class T>
QPushButton* QtDeferredElementSelectionWidget<T>::getButtonChangeSelection()
{
    return mButtonChangeSelection;
}

#endif // QTDEFERREDELEMENTSELECTIONWIDGET_CPP
