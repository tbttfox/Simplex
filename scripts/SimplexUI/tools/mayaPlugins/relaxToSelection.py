import maya.cmds as cmds
from Qt.QtWidgets import QAction

def registerTool(window, menu):
	relaxToSelectionACT = QAction("Relax To Selection", window)
	menu.addAction(relaxToSelectionACT)
	relaxToSelectionACT.triggered.connect(relaxToSelectionInterface)

def relaxToSelectionInterface():
	sel = cmds.ls(sl=True)
	if len(sel) >= 2:
		relaxToSelection(sel[0], sel[1])

def relaxToSelection(source, target):
	'''
	Transfer high-frequency sculpts (like wrinkles) from one shape to another

	This sets up a delta mush on a detailed mesh (source) before blending to a
	less detailed shape (target). Turning up the deltaMush re-applies the
	high-resolution deltas to that less detailed shape so then blend it back
	into the target
	'''
	sourceDup = cmds.duplicate(source, name="deltaMushMesh")[0]
	targetDup = cmds.duplicate(target, name="targetDup")[0]
	deltaMushRelax = cmds.group(sourceDup, name="deltaMushRelax")
	cmds.hide([sourceDup, targetDup, deltaMushRelax])

	cmds.addAttr(deltaMushRelax, longName="smooth_iter", attributeType="long", minValue=0, maxValue=100, defaultValue=10)
	smoothIter = "{0}.smooth_iter".format(deltaMushRelax)
	cmds.setAttr(smoothIter, edit=True, keyable=True)

	blender = cmds.blendShape(targetDup, sourceDup)
	deltaMush = cmds.deltaMush(sourceDup, smoothingIterations=10, smoothingStep=0.5, pinBorderVertices=1, envelope=1)[0]
	cmds.connectAttr(smoothIter, deltaMush+".smoothingIterations", force=True)

	cmds.delete(targetDup)
	finalBlend = cmds.blendShape(sourceDup, target)
	cmds.blendShape(finalBlend, edit=True, weight=((0, 1)))
	cmds.blendShape(blender, edit=True, weight=((0, 1)))

	cmds.select(deltaMushRelax)

