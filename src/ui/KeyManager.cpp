#include "KeyManager.h"

#include <QDebug>
#include <QKeySequence>

#include <iostream>

KeyManager::KeyManager()
{
    mMaxKeys = 10000;
    mKeyDowns.resize(static_cast<size_t>(mMaxKeys), false);
}

void KeyManager::keyDownEvent(QKeyEvent* event)
{
    int keyId = event->key();
    if (isKeyValid(keyId))
    {
        mKeyDowns[static_cast<size_t>(keyId)] = true;
    }

    mModifiers = event->modifiers();
}

void KeyManager::keyReleaseEvent(QKeyEvent* event)
{
    int keyId = event->key();
    if (isKeyValid(keyId))
        mKeyDowns[static_cast<size_t>(event->key())] = false;

    mModifiers = event->modifiers();
}

bool KeyManager::isKeyDown(Qt::Key key)
{
    return mKeyDowns[key];
}

bool KeyManager::isShiftDown()
{
    return mModifiers & Qt::ShiftModifier;
}

bool KeyManager::isCtrlDown()
{
    return mModifiers & Qt::ControlModifier;
}

bool KeyManager::isKeyValid(int keyId)
{
    return keyId != 0 && keyId != Qt::Key_unknown &&
            keyId > 0 && keyId < mMaxKeys;
}

KeyManager* KeyManager::mInstance = new KeyManager();
