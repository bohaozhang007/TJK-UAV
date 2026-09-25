import pickle


def reply(output, result, error=None):
    """Send a model result or error through the binary reply pipe."""
    pickle.dump((result, error), output)
    output.flush()
