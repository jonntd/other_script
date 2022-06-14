import pymel.all as pm
def fixAdvMouthCmd():
    old_upperLipCenterPlane = pm.PyNode("upperLipCenterPlane")
    old_lowerLipCenterPlane = pm.PyNode("lowerLipCenterPlane")
    lib_def_joint = ["LipRibbonJoint_L","LipRibbonJoint_R"]
    upperLipCenterPlane = pm.duplicate(old_upperLipCenterPlane)[0]
    lowerLipCenterPlane = pm.duplicate(old_lowerLipCenterPlane)[0]
    old_up_lip_joint_list = pm.skinCluster(old_upperLipCenterPlane,q=1,inf=1)
    old_lower_lip_joint_list = pm.skinCluster(old_lowerLipCenterPlane,q=1,inf=1)
    for joint in lib_def_joint:
        old_up_lip_joint_list.remove(joint)
        old_lower_lip_joint_list.remove(joint)
    up_lip_joint_list = pm.duplicate(old_up_lip_joint_list,po=1)
    lower_lip_joint_list = pm.duplicate(old_lower_lip_joint_list,po=1)
    lib_def_joint_list = pm.duplicate(lib_def_joint,po=1)
    up_lip_joint_list.insert(0,lib_def_joint_list[1])
    up_lip_joint_list.append(lib_def_joint_list[0])
    lower_lip_joint_list.insert(0,lib_def_joint_list[1])
    lower_lip_joint_list.append(lib_def_joint_list[0])
    up_LipCenterPlaneSC = pm.skinCluster(up_lip_joint_list,upperLipCenterPlane,tsb=1,dr=4,mi=3)
    lower_LipCenterPlaneSC = pm.skinCluster(lower_lip_joint_list,lowerLipCenterPlane,tsb=1,dr=4,mi=3)
    for i,joint in enumerate(up_lip_joint_list):
        pm.select((str(upperLipCenterPlane) + ".cv[" + str((6 - i)) + "][0:4]"), r=1)
        pm.skinPercent(up_LipCenterPlaneSC, tv=(joint, 1))
    for i,joint in enumerate(lower_lip_joint_list):
        pm.select((str(lowerLipCenterPlane) + ".cv[" + str((6 - i)) + "][0:4]"), r=1)
        pm.skinPercent(lower_LipCenterPlaneSC, tv=(joint, 1))
    old_up_lip_joint_list.insert(0,lib_def_joint[1])
    old_up_lip_joint_list.append(lib_def_joint[0])
    old_lower_lip_joint_list.insert(0,lib_def_joint[1])
    old_lower_lip_joint_list.append(lib_def_joint[0])
    old_upperLip_follicles = pm.listConnections(old_upperLipCenterPlane.getShape().local,s=0,d=1)
    old_lowerLip_follicles = pm.listConnections(old_lowerLipCenterPlane.getShape().local,s=0,d=1)
    for follicle in old_upperLip_follicles:
        nfo = pm.duplicate(follicle.getShape())[0]
        nfo.getShape().outRotate >> nfo.rotate
        nfo.getShape().outTranslate >> nfo.translate
        pm.connectAttr(upperLipCenterPlane.getShape().local,nfo.inputSurface)
        pm.connectAttr("LipFollicles.worldInverseMatrix[0]",nfo.inputWorldMatrix)
        ocn = pm.listConnections(follicle.rotate,s=0,d=1)[0]
        nfo.parentMatrix[0] >> ocn.target[0].targetParentMatrix;
        nfo.rotateOrder >>  ocn.target[0].targetRotateOrder;
        nfo.rotate >>  ocn.target[0].targetRotate;
    for follicle in old_lowerLip_follicles:
        nfo = pm.duplicate(follicle.getShape())[0]
        nfo.getShape().outRotate >> nfo.rotate
        nfo.getShape().outTranslate >> nfo.translate
        pm.connectAttr(lowerLipCenterPlane.getShape().local,nfo.inputSurface)
        pm.connectAttr("LipFollicles.worldInverseMatrix[0]",nfo.inputWorldMatrix)
        ocn = pm.listConnections(follicle.rotate,s=0,d=1)[0]
        nfo.parentMatrix[0] >> ocn.target[0].targetParentMatrix;
        nfo.rotateOrder >>  ocn.target[0].targetRotateOrder;
        nfo.rotate >>  ocn.target[0].targetRotate;
    for a,b in zip (up_lip_joint_list,old_up_lip_joint_list):
        # pm.connectAttr(pm.PyNode(b).rotate,a.rotate,f=1)
        pm.orientConstraint(b,a,mo=1)
    for a,b in zip (lower_lip_joint_list,old_lower_lip_joint_list):
        # pm.connectAttr(pm.PyNode(b).rotate,a.rotate,f=1)
        pm.orientConstraint(b,a,mo=1)
    for a in list(set(up_lip_joint_list + lower_lip_joint_list)):
        pm.parentConstraint("Head_M",a,mo=1, weight=1, skipRotate=['x', 'y', 'z'])
