#ifndef KEYMANAGER_H
#define KEYMANAGER_H

#include <QKeyEvent>

// Manages all key input.
// -> Use isKeyDown() to find out if a certain button is currently pressed.
// -> Use getModifiers() or is(Shift/Ctrl)Down() to find out if shift or ctrl
//      is pressed.
class KeyManager
{
private:
    KeyManager();

public:
    static KeyManager* instance()
    {
        return mInstance;
    }

    void keyPressEvent(QKeyEvent *event);

    void keyReleaseEvent(QKeyEvent *event);

    bool isKeyDown(Qt::Key key);

    // Can be used to detect if shift / ctrl is pressed by:
    // if (getModifiers() & Qt::ShiftModifier) // shift pressed
    // if (getModifiers() & Qt::ControlModifier) // ctrl pressed
    Qt::KeyboardModifiers getModifiers();

    // Returns true if shift is currently pressed.
    bool isShiftDown();

    // Returns true if ctrl is currently pressed.
    bool isCtrlDown();

private:

    // Checks if a key with the given id is valid. Its not valid if its
    // id is 0, Qt::key_unknown, or the id exceeds the maximum number of
    // supported keys in this class (currently 10000).
    bool isKeyValid(int keyId);

    static KeyManager* mInstance;

    std::vector<bool> mKeyDowns;

    Qt::KeyboardModifiers mModifiers;

    int mMaxKeys;
};

#endif // KEYMANAGER_H
