package commands;

class CommandState {

// Whether or not it is interruptible.
private final boolean m_interruptible;


CommandState(boolean interruptible) {
    m_interruptible = interruptible;
}

boolean isInterruptible() {
    return m_interruptible;
}

}