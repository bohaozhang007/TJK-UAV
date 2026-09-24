# Code style

For Python code in this directory and its subdirectories, use nested `with`
statements when multiple context managers are needed. Each `with` or `async with`
statement must contain only one context manager. Preserve their original order.
