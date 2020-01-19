#ifndef QTDEFERREDELEMENTSELECTIONWIDGET_H
#define QTDEFERREDELEMENTSELECTIONWIDGET_H

#include <QComboBox>
#include <QPushButton>
#include <QWidget>

// Enables the deferred selection of an element of type T.
//
// Adds a combo box and a button:
//
// - For as long as the button is pressed (stays pressed), any call
// of the selectElements() method causes the button to uncheck and adds the
// selected objets to the combo box.
//
// - The combo box allows to chose one of the scenes nodes that were selected.
//      Use getSelectedElement() to retrieve that element.
//      Use getSelectedElements() to retrieve all elements, independently of
//      what is chosen in the combo box.
//
// - Adds another button to change the selection.
//
template <class T>
class QtDeferredElementSelectionWidget : public QWidget
{
public:

    // \param addChangeSelectionButton - adds an additional button to the right
    //      for changing the global selection. If true, use
    //      getButtonChangeSelection() to access this button.
    QtDeferredElementSelectionWidget(
            QWidget* parent = nullptr, bool addChangeSelectionButton = true);

    virtual ~QtDeferredElementSelectionWidget() override;

    virtual void selectElements(
            const std::vector<T>& selectedElements,
            const std::vector<std::string>& names);

    // Select the elements if the button that enables selection is pressed.
    // Unchecks the button (with signales blocked).
    virtual void selectElementsIfButtonPressed(
            const std::vector<T>& selectedElements,
            const std::vector<std::string>& names);

    // Updates the names of all elements.
    // \param names - must be of size of the selected elements that were passed
    //      to selectElements().
    void updateNames(const std::vector<std::string>& names);

    // Updates the name of the element at the given index.
    void updateName(size_t index, const std::string& name);

    // Returns the index that the element has in the combo box.
    // Returns 0 if there is none.
    size_t getIndex(T element);

    // Returns the selected element that is chosen in the combo box.
    T getSelectedElement() const;

    // Returns all selected elements, independently of the combo box.
    std::vector<T> getSelectedElements() const;

protected:
    QPushButton* getButtonSetSelection();

    // If addChangeSelectionButton was set true, returns the button, else,
    // returns nullptr.
    QPushButton* getButtonChangeSelection();

private:
    QComboBox* mComboBox;
    QPushButton* mButtonAddSelection;
    QPushButton* mButtonChangeSelection;

    std::vector<T> mSelectedElements;
};

#include "QtDeferredElementSelectionWidget.cpp"

#endif // QTDEFERREDELEMENTSELECTIONWIDGET_H
