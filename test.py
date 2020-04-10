import moveit.task_constructor as mtc

o=mtc.MyObject(42)
a=o
#print o
#print a
mtc.access(o)
mtc.access(o)
mtc.consume(o)
mtc.access(o)
mtc.access(o)
#print o
#print a
del o
del a
print "done"
