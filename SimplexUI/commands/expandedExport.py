# Copyright 2016, Blur Studio
#
# This file is part of Simplex.
#
# Simplex is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# Simplex is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with Simplex.  If not, see <http://www.gnu.org/licenses/>.

from alembic.AbcGeom import OXform, OPolyMesh, OPolyMeshSchemaSample, OV2fGeomParamSample, GeometryScope
from alembic.Abc import OArchive, OStringProperty

from ..items import Slider, Combo, Traversal
from ..interface.mayaInterface import disconnected
from .alembicCommon import mkSampleVertexPoints
from .. import OGAWA

import maya.OpenMaya as om
from imath import V2fArray, V3fArray, IntArray, UnsignedIntArray
from ctypes import c_float
import numpy as np
from pysimplex import PySimplex

def getShape(mesh):
	'''Get the np.array shape of the mesh connected to the smpx

	Parameters
	----------
	mesh :
		

	Returns
	-------

	'''
	# This should probably be rolled into the DCC code
	sl = om.MSelectionList()
	sl.add(mesh)
	thing = om.MDagPath()
	sl.getDagPath(0, thing)
	meshFn = om.MFnMesh(thing)
	rawPts = meshFn.getRawPoints()
	ptCount = meshFn.numVertices()
	cta = (c_float * ptCount * 3).from_address(int(rawPts))
	out = np.ctypeslib.as_array(cta)
	out = np.copy(out)
	out = out.reshape((-1, 3))
	return out

def getAbcFaces(mesh):
	'''

	Parameters
	----------
	mesh :
		

	Returns
	-------

	'''
	# Get the MDagPath from the name of the mesh
	sl = om.MSelectionList()
	sl.add(mesh)
	thing = om.MDagPath()
	sl.getDagPath(0, thing)
	meshFn = om.MFnMesh(thing)

	faces = []
	faceCounts = []
	#uvArray = []
	uvIdxArray = []
	vIdx = om.MIntArray()

	util = om.MScriptUtil()
	util.createFromInt(0)
	uvIdxPtr = util.asIntPtr()
	uArray = om.MFloatArray()
	vArray = om.MFloatArray()
	meshFn.getUVs(uArray, vArray)
	hasUvs = uArray.length() > 0

	for i in range(meshFn.numPolygons()):
		meshFn.getPolygonVertices(i, vIdx)
		face = []
		for j in reversed(xrange(vIdx.length())):
			face.append(vIdx[j])
			if hasUvs:
				meshFn.getPolygonUVid(i, j, uvIdxPtr)
				uvIdx = util.getInt(uvIdxPtr)
				if uvIdx >= uArray.length() or uvIdx < 0:
					uvIdx = 0
				uvIdxArray.append(uvIdx)

		face = [vIdx[j] for j in reversed(xrange(vIdx.length()))]
		faces.extend(face)
		faceCounts.append(vIdx.length())

	abcFaceIndices = IntArray(len(faces))
	for i in xrange(len(faces)):
		abcFaceIndices[i] = faces[i]

	abcFaceCounts = IntArray(len(faceCounts))
	for i in xrange(len(faceCounts)):
		abcFaceCounts[i] = faceCounts[i]

	if hasUvs:
		abcUVArray = V2fArray(len(uArray))
		for i in xrange(len(uArray)):
			abcUVArray[i] = (uArray[i], vArray[i])
		abcUVIdxArray = UnsignedIntArray(len(uvIdxArray))
		for i in xrange(len(uvIdxArray)):
			abcUVIdxArray[i] = uvIdxArray[i]
		uv = OV2fGeomParamSample(abcUVArray, abcUVIdxArray, GeometryScope.kFacevaryingScope)
	else:
		uv = None

	return abcFaceIndices, abcFaceCounts, uv

def _setSliders(ctrl, val, svs):
	'''

	Parameters
	----------
	ctrl :
		
	val :
		
	svs :
		

	Returns
	-------

	'''
	slis, vals = svs.setdefault(ctrl.simplex, ([], []))

	if isinstance(ctrl, Slider):
		slis.append(ctrl)
		vals.append(val)
	elif isinstance(ctrl, Combo):
		for cp in ctrl.pairs:
			slis.append(cp.slider)
			vals.append(cp.value * abs(val))
	elif isinstance(ctrl, Traversal):
		# set the ctrl.multiplierCtrl.controller to 1
		multCtrl = ctrl.multiplierCtrl.controller
		multVal = ctrl.multiplierCtrl.value if val != 0.0 else 0.0
		progCtrl = ctrl.progressCtrl.controller
		progVal = ctrl.progressCtrl.value
		_setSliders(multCtrl, multVal, svs)
		_setSliders(progCtrl, val, svs)

def setSliderGroup(ctrls, val):
	'''

	Parameters
	----------
	ctrls :
		
	val :
		

	Returns
	-------

	'''
	svs = {}
	for ctrl in ctrls:
		_setSliders(ctrl, val, svs)

	for smpx, (slis, vals) in svs.iteritems():
		smpx.setSlidersWeights(slis, vals)

def clientPartition(master, clients):
	'''

	Parameters
	----------
	master :
		
	clients :
		

	Returns
	-------

	'''
	sliders, combos, traversals = {}, {}, {}
	for cli in [master] + clients:
		for sli in cli.sliders:
			sliders.setdefault(sli.name, []).append(sli)

		for com in cli.combos:
			combos.setdefault(com.name, []).append(com)

		for trav in cli.traversals:
			traversals.setdefault(trav.name, []).append(trav)
	return sliders, combos, traversals

def zeroAll(smpxs):
	'''

	Parameters
	----------
	smpxs :
		

	Returns
	-------

	'''
	for smpx in smpxs:
		smpx.setSlidersWeights(smpx.sliders, [0.0] * len(smpx.sliders))

def getExpandedData(master, clients, mesh):
	'''Get the fully expanded shape data for each slider, combo, and traversal
		at each of its underlying shapes

	Parameters
	----------
	master :
		
	clients :
		
	mesh :
		

	Returns
	-------

	'''
	# zero everything
	zeroAll([master] + clients)
	sliPart, cmbPart, travPart = clientPartition(master, clients)
	sliderShapes = {}
	for slider in master.sliders:
		ss = {}
		sliderShapes[slider] = ss
		for pp in slider.prog.pairs:
			if pp.shape.isRest:
				continue
			setSliderGroup(sliPart[slider.name], pp.value)
			ss[pp] = getShape(mesh)
			setSliderGroup(sliPart[slider.name], 0.0)

	# Disable traversals
	zeroAll([master] + clients)
	travShapeThings = []
	comboShapes = {}
	for trav in master.traversals:
		for pp in trav.prog.pairs:
			if pp.shape.isRest:
				continue
			travShapeThings.append(pp.shape)
	travShapeThings = [i.thing for i in travShapeThings]

	# Get the combos with traversals disconnected
	with disconnected(travShapeThings):
		for combo in master.combos:
			ss = {}
			comboShapes[combo] = ss
			for pp in combo.prog.pairs:
				if pp.shape.isRest:
					continue
				setSliderGroup(cmbPart[combo.name], pp.value)
				ss[pp] = getShape(mesh)
				setSliderGroup(cmbPart[combo.name], 0.0)

	zeroAll([master] + clients)
	travShapes = {}
	for trav in master.traversals:
		ss = {}
		travShapes[trav] = ss
		progCtrl = trav.progressCtrl.controller
		progVal = trav.progressCtrl.value
		for pp in progCtrl.prog.pairs:
			if pp.shape.isRest:
				continue
			if pp.value * progVal <= 0.0:
				# Traversals only activate if the progression is in the same
				# pos/neg direction as the value. Otherwise we just skip
				continue

			setSliderGroup(travPart[trav.name], pp.value)
			ss[pp] = getShape(mesh)
			setSliderGroup(travPart[trav.name], 0.0)

	zeroAll([master] + clients)
	restShape = getShape(mesh)

	return restShape, sliderShapes, comboShapes, travShapes

def _setInputs(inVec, item, indexBySlider, value):
	'''Being clever
		Sliders or Combos just set the value and return
		Traversals recursively call this function with the controllers (that only either sliders or combos)

	Parameters
	----------
	inVec :
		
	item :
		
	indexBySlider :
		
	value :
		

	Returns
	-------

	'''
	if isinstance(item, Slider):
		inVec[indexBySlider[item]] = value
		return inVec
	elif isinstance(item, Combo):
		for pair in item.pairs:
			inVec[indexBySlider[pair.slider]] = pair.value * abs(value)
		return inVec
	elif isinstance(item, Traversal):
		inVec = _setInputs(inVec, item.multiplierCtrl.controller, indexBySlider, item.multiplierCtrl.value)
		inVec = _setInputs(inVec, item.progressCtrl.controller, indexBySlider, item.progressCtrl.value * value)
		return inVec
	raise ValueError("Not a Slider, Combo, or Traversal. Got type {0}: {1}".format(type(item), item))

def _buildSolverInputs(simplex, item, value, indexBySlider):
	'''Build an input vector for the solver that will
		produce a required progression value on an item

	Parameters
	----------
	simplex :
		
	item :
		
	value :
		
	indexBySlider :
		

	Returns
	-------

	'''
	inVec = [0.0] * len(simplex.sliders)
	return _setInputs(inVec, item, indexBySlider, value)

def getTravDepth(trav):
	'''

	Parameters
	----------
	trav :
		

	Returns
	-------

	'''
	inputs = []
	mult = trav.multiplierCtrl.controller
	prog = trav.progressCtrl.controller
	for item in (mult, prog):
		if isinstance(item, Slider):
			inputs.append(item)
		elif isinstance(item, Combo):
			for cp in item.pairs:
				inputs.append(cp.slider)
	return len(set(inputs))

def parseExpandedData(smpx, restShape, sliderShapes, comboShapes, travShapes):
	'''Turn the expanded data into shapeDeltas connected to the actual Shape objects

	Parameters
	----------
	smpx :
		
	restShape :
		
	sliderShapes :
		
	comboShapes :
		
	travShapes :
		

	Returns
	-------

	'''
	solver = PySimplex(smpx.dump())
	shapeArray = np.zeros((len(smpx.shapes), len(restShape), 3))

	indexBySlider = {s: i for i, s in enumerate(smpx.sliders)}
	indexByShape = {s: i for i, s in enumerate(smpx.shapes)}

	floatShapeSet = set(smpx.getFloatingShapes())
	floatIdxs = sorted(set([indexByShape[s] for s in floatShapeSet]))
	travShapeSet = set([pp.shape for t in smpx.traversals for pp in t.prog.pairs])
	travIdxs = sorted(set([indexByShape[s] for s in travShapeSet]))

	# Sliders are simple, just set their shapes directly
	for ppDict in sliderShapes.itervalues():
		for pp, shp in ppDict.iteritems():
			shapeArray[indexByShape[pp.shape]] = shp - restShape

	# First sort the combos by depth
	comboByDepth = {}
	for combo in smpx.combos:
		comboByDepth.setdefault(len(combo.pairs), []).append(combo)

	for depth in sorted(comboByDepth.keys()):
		for combo in comboByDepth[depth]:
			for pp, shp in comboShapes[combo].iteritems():
				inVec = _buildSolverInputs(smpx, combo, pp.value, indexBySlider)
				outVec = np.array(solver.solve(inVec))
				outVec[np.where(np.isclose(outVec, 0.0))] = 0.0
				outVec[np.where(np.isclose(outVec, 1.0))] = 1.0
				outVec[indexByShape[pp.shape]] = 0.0

				# ignore any traversals
				outVec[travIdxs] = 0.0

				# ignore floaters if we're not currently checking floaters
				if pp.shape not in floatShapeSet:
					outVec[floatIdxs] = 0.0

				# set the shape delta to the output
				baseShape = np.dot(outVec, shapeArray.swapaxes(0, 1))
				shapeArray[indexByShape[pp.shape]] = shp - restShape - baseShape

	# First the traversals by depth
	travByDepth = {}
	for trav in smpx.traversals:
		travByDepth.setdefault(getTravDepth(trav), []).append(trav)

	for depth in sorted(travByDepth.keys()):
		for trav in travByDepth[depth]:
			for pp, shp in travShapes[trav].iteritems():
				inVec = _buildSolverInputs(smpx, trav, pp.value, indexBySlider)
				outVec = np.array(solver.solve(inVec))
				outVec[np.where(np.isclose(outVec, 0.0))] = 0.0
				outVec[np.where(np.isclose(outVec, 1.0))] = 1.0
				outVec[indexByShape[pp.shape]] = 0.0

				# set the shape delta to the output
				baseShape = np.dot(outVec, shapeArray.swapaxes(0, 1))
				shapeArray[indexByShape[pp.shape]] = shp - restShape - baseShape
	return shapeArray

def buildShapeArray(mesh, master, clients):
	'''

	Parameters
	----------
	mesh :
		
	master :
		
	clients :
		

	Returns
	-------

	'''
	restShape, sliderShapes, comboShapes, travShapes = getExpandedData(master, clients, mesh)
	shapeArray = parseExpandedData(master, restShape, sliderShapes, comboShapes, travShapes)
	shapeArray += restShape[None, ...]
	return shapeArray

def _exportAbc(arch, smpx, shapeArray, faces, counts, uvs):
	'''

	Parameters
	----------
	arch :
		
	smpx :
		
	shapeArray :
		
	faces :
		
	counts :
		
	uvs :
		

	Returns
	-------

	'''
	par = OXform(arch.getTop(), str(smpx.name))
	props = par.getSchema().getUserProperties()
	prop = OStringProperty(props, "simplex")
	prop.setValue(str(smpx.dump()))
	abcMesh = OPolyMesh(par, str(smpx.name))
	schema = abcMesh.getSchema()
	for i in range(len(smpx.shapes)):
		if uvs is not None:
			abcSample = OPolyMeshSchemaSample(mkSampleVertexPoints(shapeArray[i]), faces, counts, uvs)
		else:
			# can't just pass uvs as None because of how the Alembic API works
			abcSample = OPolyMeshSchemaSample(mkSampleVertexPoints(shapeArray[i]), faces, counts)
		schema.set(abcSample)

def expandedExportAbc(path, mesh, master, clients=()):
	'''Export the alembic by re-building the deltas from all of the full shapes
		This is required for delta-mushing a system, because the sum of mushed shapes
		is not the same as the mushed sum-of-shapes

	Parameters
	----------
	path :
		
	mesh :
		
	master :
		
	clients :
		 (Default value = ())

	Returns
	-------

	'''
	# Convert clients to a list if need be
	if not clients:
		clients = []
	elif not isinstance(clients, list):
		if isinstance(clients, tuple):
			clients = list(clients)
		else:
			clients = [clients]

	faces, counts, uvs = getAbcFaces(mesh)
	shapeArray = buildShapeArray(mesh, master, clients)

	# export the data to alembic
	arch = OArchive(str(path), OGAWA) # alembic does not like unicode filepaths
	try:
		_exportAbc(arch, master, shapeArray, faces, counts, uvs)
	finally:
		del arch


if __name__ == "__main__":
	# get the smpx from the UI
	master = Simplex.buildSystemFromMesh('Face_SIMPLEX', 'Face')
	client = Simplex.buildSystemFromMesh('Face_SIMPLEX2', 'Face2')
	outPath = r'D:\Users\tyler\Desktop\TEST\expanded.smpx'
	expandedExportAbc(outPath, 'Face_SIMPLEX', master, client)
