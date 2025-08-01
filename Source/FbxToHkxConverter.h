/*
 *
 * Confidential Information of Telekinesys Research Limited (t/a Havok). Not for disclosure or distribution without Havok's
 * prior written consent. This software contains code, techniques and know-how which is confidential and proprietary to Havok.
 * Product and Trade Secret source code contains trade secrets of Havok. Havok Software (C) Copyright 1999-2014 Telekinesys Research Limited t/a Havok. All Rights Reserved. Use of this software is subject to the terms of an end user license agreement.
 *
 */

#ifndef HK_FBXTOHKX_CONVERTER
#define HK_FBXTOHKX_CONVERTER

#define FBXSDK_NEW_API

#pragma warning(push,3)
#include <fbxsdk.h>
#pragma warning(pop)

#include <Common/Base/hkBase.h>
#include <Common/SceneData/Scene/hkxScene.h>
#include <Common/SceneData/Graph/hkxNode.h>
#include <Common/Base/Container/PointerMap/hkPointerMap.h>
#include <Common/Base/Container/String/Deprecated/hkStringOld.h>

class FbxToHkxConverter
{
public:

	struct Options
	{
		FbxManager* m_fbxSdkManager;
		bool		m_exportMeshes;
		bool		m_exportMaterials;
		bool		m_exportAttributes;
		bool		m_exportAnnotations;
		bool		m_exportLights;
		bool		m_exportCameras;
		bool		m_exportSplines;
		bool		m_exportVertexTangents;
		bool		m_exportVertexAnimations;
		bool		m_visibleOnly;
		bool		m_selectedOnly;
		bool		m_storeKeyframeSamplePoints;

		Options(FbxManager* fbxSdkManager);
	};

	FbxToHkxConverter(const Options& options);
	~FbxToHkxConverter();
	
	bool createScenes(FbxScene* fbxScene, bool noTakes, const char* path);
	void saveScenes(const char *path, const char *name);
private:

	//---- static declarations
	
	static FbxAMatrix convertMatrix(const FbxMatrix& mat);

	template<typename FbxMatrixType>
	static void convertFbxXMatrixToMatrix4(const FbxMatrixType& fbxMatrix, hkMatrix4& matrix)
	{
		FbxVector4 v0 = fbxMatrix.GetRow(0);
		FbxVector4 v1 = fbxMatrix.GetRow(1);
		FbxVector4 v2 = fbxMatrix.GetRow(2);
		FbxVector4 v3 = fbxMatrix.GetRow(3);

		hkVector4 c0; c0.set((float)v0[0],(float)v0[1],(float)v0[2],(float)v0[3]);
		hkVector4 c1; c1.set((float)v1[0],(float)v1[1],(float)v1[2],(float)v1[3]);
		hkVector4 c2; c2.set((float)v2[0],(float)v2[1],(float)v2[2],(float)v2[3]);
		hkVector4 c3; c3.set((float)v3[0],(float)v3[1],(float)v3[2],(float)v3[3]);

		matrix.setCols(c0,c1,c2,c3);
	}


	static void fillBuffers(
		FbxMesh* pMesh,
		FbxNode* originalNode,
		hkxVertexBuffer* newVB,
		hkxIndexBuffer* newIB,
		const hkArray<float>& skinControlPointWeights,
		const hkArray<int>& skinIndicesToClusters,
		const hkArray<int>& polyIndices);
	static void findChildren(FbxNode* root, hkArray<FbxNode*>& children, FbxNodeAttribute::EType type);

	// Get the global position of the node for the current pose.
	// If the specified node is not part of the pose or no pose is specified, get its
	// global position at the current time.
	static FbxAMatrix getGlobalPosition(
		FbxNode* pNode,
		const FbxTime& pTime,
		FbxPose* pPose,
		FbxAMatrix* pParentGlobalPosition = NULL);

	//---- declarations

	void clear();

	bool createSceneStack(int animStackIndex, const char* path = NULL);
	void addNodesRecursive(hkxScene *scene, FbxNode* fbxNode, hkxNode* node, int animStackIndex, const char* path = NULL);	
	void addMesh(hkxScene *scene, FbxNode* meshNode, hkxNode* node, const char* path = NULL);
	void addCamera(hkxScene *scene, FbxNode* cameraNode, hkxNode* node);
	void addLight(hkxScene *scene, FbxNode* lightNode, hkxNode* node);
	void addSpline(hkxScene *scene, FbxNode* splineNode, hkxNode* node);
	hkxMaterial* createMaterial(FbxSurfaceMaterial* lMaterial, FbxMesh* pMesh, hkxScene* scene);
	void getMaterialsInMesh(FbxMesh* pMesh, hkArray<FbxSurfaceMaterial*>& materialsOut);

	void extractKeyFramesAndAnnotations(hkxScene *scene, FbxNode* fbxChildNode, hkxNode* newChildNode, int animStackIndex);

	// Convert an FBX texture into a Havok texture type. This might return the cached result from a prior conversion.
	hkReferencedObject* convertTexture(
		hkxScene *scene,
		FbxSurfaceMaterial* material,
		const FbxStringList& uvSetNames,
		hkxMaterial* mat,
		const char* fbxTextureType,
		int& uvSetIndex);

	void convertTextures(
		hkxScene *scene,
		FbxSurfaceMaterial* fbxMat,
		const FbxStringList& uvSetNames,
		hkxMaterial* mat);
	void convertTexture(
		hkxScene *scene,
		FbxSurfaceMaterial* fbxMat,
		const FbxStringList& uvSetNames,
		hkxMaterial* mat,
		const char* textureTypeName,
		hkxMaterial::TextureType textureType);
	
	void addSampledNodeAttributeGroups(
		hkxScene *scene,
		int animStackIndex,
		FbxObject* fbxObject,
		hkxAttributeHolder* hkx_attributeHolder,
		bool recurse = true);
	bool createAndSampleAttribute(
		hkxScene *scene,
		int animStackIndex,
		FbxProperty& property,
		hkxAttribute& hkx_attribute);

	//---- member variables

	Options m_options;

	hkArray<hkxScene*> m_scenes;
	FbxScene *m_curFbxScene;
	FbxPose *m_pose;
	hkStringBuf m_modeller;
	int m_numAnimStacks;
	int m_numBones;
	FbxTime m_startTime;
	FbxNode *m_rootNode;

	// A cache of converted FBX -> Havok textures
	hkPointerMap<FbxTexture*, hkRefVariant*> m_convertedTextures;
	// A cache of converted FBX -> Havok materials
	hkPointerMap<FbxSurfaceMaterial*, hkxMaterial*> m_convertedMaterials;
};

#endif

/*
 * Havok SDK - NO SOURCE PC DOWNLOAD, BUILD(#20140907)
 * 
 * Confidential Information of Havok.  (C) Copyright 1999-2014
 * Telekinesys Research Limited t/a Havok. All Rights Reserved. The Havok
 * Logo, and the Havok buzzsaw logo are trademarks of Havok.  Title, ownership
 * rights, and intellectual property rights in the Havok software remain in
 * Havok and/or its suppliers.
 * 
 * Use of this software for evaluation purposes is subject to and indicates
 * acceptance of the End User licence Agreement for this product. A copy of
 * the license is included with this software and is also available at www.havok.com/tryhavok.
 * 
 */
