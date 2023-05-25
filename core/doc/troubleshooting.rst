.. _sec-troubleshooting:

Troubleshooting
===============

``Task::init()`` reports mismatching interfaces
-----------------------------------------------

Before planning, the planning pipeline is checked for consistency. Each stage type has a specific flow interface, e.g. generator-type stages write into both, their begin and end interface, propagator-type stages read from one and write to the opposite, and connector-type stages read from both sides. Obviously these interfaces need to match. If they don't, an ``InitStageException`` is thrown. Per default, this is not very verbose, but you can use the following code snippet to get some more info:

	.. code-block:: c

		try {
			task.plan();
		} catch (const InitStageException& e) {
			std::cerr << e << std::endl;
			throw;
		}

For example, the following pipeline:

- ↕ current
- ⛓ connect
- ↑ moveTo

throws the error: ``task pipeline: end interface (←) of 'moveto' does not match mine (→)``.

The validation process runs sequentially through a ``SerialContainer``. Here, ``current``, as a generator stage is compatible to ``connect``, writing to the interface read by ``connect``.
``moveTo`` as a propagator can operate, in principle, forwards and backwards. By default, the operation direction is inferred automatically. Here, as ``connect`` requires a reading end-interface, moveTo should seemingly operate backwards. However, now the whole pipeline is incompatible to the enclosing container's interface: a task requires a generator-type interface as it generates solutions - there is no input interface to read from. Hence, the *reading* end interface (←) of ``moveto`` conflicts with a *writing* end interface of the task.

Obviously, in a ``ParallelContainer`` all (direct) children need to share a common interface.
