.. _subsec-tut-properties:

Properties
----------

Properties are named attributes of a stage.
They can be used to configure the stages behaviour
and control further substages. Lets take a closer
look at how to work with properties.

Basic Operations with Properties
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Lets define a property and assign a description, as well as
a value to it.

.. literalinclude:: ../../../demo/scripts/properties.py
    :language: python
    :start-after: [propertyTut1]
    :end-before: [propertyTut1]

Notice that a property always has two values: the current value
and the default value. Before we use the property, we might want to
check if the current value defined.

.. literalinclude:: ../../../demo/scripts/properties.py
    :language: python
    :start-after: [propertyTut2]
    :end-before: [propertyTut2]

Now we are ready to safely retrieve the values of the proprty!

.. literalinclude:: ../../../demo/scripts/properties.py
    :language: python
    :start-after: [propertyTut3]
    :end-before: [propertyTut3]

The Property Map
^^^^^^^^^^^^^^^^

Usually a stage comprises multiple properties. A stage
contains a single PropertyMap that acts as a container for
all properties associated to that stage.

Lets first create a PropertyMap in isolation and initialize
some properties using a dict. As you can see, properties can be
of arbitrary type.

.. literalinclude:: ../../../demo/scripts/properties.py
    :language: python
    :start-after: [propertyTut4]
    :end-before: [propertyTut4]

Properties can also be initialized using a more pythonic way.

.. literalinclude:: ../../../demo/scripts/properties.py
    :language: python
    :start-after: [propertyTut5]
    :end-before: [propertyTut5]

There are two ways to retrieve properties back from the property map.
We might only be interested in in the value of the property:

.. literalinclude:: ../../../demo/scripts/properties.py
    :language: python
    :start-after: [propertyTut6]
    :end-before: [propertyTut6]

Or we can obtain a reference to the whole property object.

.. literalinclude:: ../../../demo/scripts/properties.py
    :language: python
    :start-after: [propertyTut7]
    :end-before: [propertyTut7]

The PropertyMap class additionally provides an iterator that can be used in loops.

.. literalinclude:: ../../../demo/scripts/properties.py
    :language: python
    :start-after: [propertyTut8]
    :end-before: [propertyTut8]

Remember that wer initialized our PropertyMap by using a dict. In fact, you
can also use an existing PropertMap to copy over some properties.

.. literalinclude:: ../../../demo/scripts/properties.py
    :language: python
    :start-after: [propertyTut9]
    :end-before: [propertyTut9]

Accessing Properties of a Stage
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You can obtain a reference to the the PropertyMap of a stage like so

.. literalinclude:: ../../../demo/scripts/properties.py
    :language: python
    :start-after: [propertyTut10]
    :end-before: [propertyTut10]


As mentioned, each stage contains a PropertyMap.
Stages communicate to each other via their interfaces.
If you want to forward properties through these interfaces,
you can use the reference of a stages' property object.

.. literalinclude:: ../../../demo/scripts/compute_ik.py
    :language: python
    :start-after: [propertyTut12]
    :end-before: [propertyTut12]

.. literalinclude:: ../../../demo/scripts/compute_ik.py
    :language: python
    :start-after: [propertyTut13]
    :end-before: [propertyTut13]

.. literalinclude:: ../../../demo/scripts/compute_ik.py
    :language: python
    :start-after: [propertyTut14]
    :end-before: [propertyTut14]

Take a look at the :ref:`How-To-Guides <subsubsec-howto-compute-ik>`
for a full example of this.
