/*
 *
 * Confidential Information of Telekinesys Research Limited (t/a Havok). Not for disclosure or distribution without Havok's
 * prior written consent. This software contains code, techniques and know-how which is confidential and proprietary to Havok.
 * Product and Trade Secret source code contains trade secrets of Havok. Havok Software (C) Copyright 1999-2014 Telekinesys Research Limited t/a Havok. All Rights Reserved. Use of this software is subject to the terms of an end user license agreement.
 *
 */

// Havok base infrastructure
#include <Common/Base/hkBase.h>
#include <Common/Base/Math/hkMath.h>

#define HK_CLASSES_FILE <Common/Serialize/Classlist/hkClasses.h>

#include <Common/Base/Config/hkProductFeaturesNoPatchesOrCompat.h>
#include <Common/Base/Config/hkProductFeatures.cxx>
#include <Common/Base/System/Io/OptionParser/hkOptionParser.h>

// Keycode
#include <Common/Base/keycode.cxx>

#include <Common/Base/Ext/hkBaseExt.h>
#include <Common/Base/System/hkBaseSystem.h>
#include <Common/Base/Memory/System/hkMemorySystem.h>
#include <Common/Base/Memory/System/Util/hkMemoryInitUtil.h>
#include <Common/Base/Memory/Allocator/Malloc/hkMallocAllocator.h>
#include <Common/Base/System/Error/hkError.h>
#include <Common/SceneData/Mesh/hkxMesh.h>

#include "FbxToHkxConverter.h"

static void HK_CALL havokErrorReport(const char* msg, void*)
{
	// Output to console
	printf("%s\n", msg);
}

int main(int argc, char* argv[])
{
	// initialize Havok internals
	{
		hkMemorySystem::FrameInfo frameInfo(0);

#ifdef _DEBUG
		// (Use debug mem manager to detect mem leaks in Havok code)
		hkMemoryRouter* memoryRouter = hkMemoryInitUtil::initChecking(hkMallocAllocator::m_defaultMallocAllocator, frameInfo);
#else
		hkMemoryRouter* memoryRouter = hkMemoryInitUtil::initFreeListLargeBlock(hkMallocAllocator::m_defaultMallocAllocator, frameInfo);
#endif

		hkBaseSystem::init( memoryRouter, havokErrorReport );

		hkError& errorhandler = hkError::getInstance();
		errorhandler.enableAll();
	}

	bool noTakes = false;
	const char* inputFile = NULL;
	const char* outputFile = NULL;
	// Parse command line
	hkOptionParser parser("FBXImporter", "Converts an fbx file into a havok tagfile (.hkt)");
	{
		hkOptionParser::Option options[] = 
		{
			hkOptionParser::Option("t", "noTakes", "if set, the first animation take is stored in input.hkt and additional takes are ignored.", &noTakes, false),
			hkOptionParser::Option("o", "output", "the absolute path to the output filename. If left unspecified, the input filename is used instead with a changed extension.", &outputFile)
		};

		if (parser.setOptions(options, HK_COUNT_OF(options)))
		{
			parser.setArguments("input.fbx", "input FBX file that is converted to hkt.", hkOptionParser::ARGUMENTS_ONE, &inputFile, 1);
			hkOptionParser::ParseResult result = parser.parse(argc, const_cast<const char**>(&argv[0]));
			if (result != hkOptionParser::PARSE_SUCCESS)
			{
				return -1;
			}
		}
	}

	// Load FBX and save as HKX
	{
		hkStringBuf filename = inputFile;
		filename.pathNormalize();

		FbxManager* fbxSdkManager = FbxManager::Create();
		if( !fbxSdkManager )
		{
			HK_ERROR(0x5213afed, "Unable to create FBX Manager!\n");
			return -1;
		}

		FbxIOSettings* fbxIoSettings = FbxIOSettings::Create(fbxSdkManager, IOSROOT);
		fbxSdkManager->SetIOSettings(fbxIoSettings);

		FbxImporter* fbxImporter = FbxImporter::Create(fbxSdkManager,"");

		if (!fbxImporter->Initialize(filename, -1, fbxSdkManager->GetIOSettings()))
		{
			HK_WARN(0x5216afed, "Failed to initialize the importer! Please ensure file " << filename << " exists\n");
			fbxSdkManager->Destroy();
			return -1;
		}

		FbxScene* fbxScene = FbxScene::Create(fbxSdkManager,"tempScene");
		if (!fbxScene)
		{
			HK_ERROR(0x5216afed, "Failed to create the scene!\n");
			fbxImporter->Destroy();
			fbxSdkManager->Destroy();
			return -1;
		}

		fbxImporter->Import(fbxScene);
		fbxImporter->Destroy();

		// Currently assume that the file is loaded from 3dsmax
		FbxAxisSystem::Max.ConvertScene(fbxScene);

		FbxToHkxConverter::Options options(fbxSdkManager);
		FbxToHkxConverter converter(options);

		if(converter.createScenes(fbxScene, noTakes))
		{
			hkStringBuf path;
			hkStringBuf name;
			// Was an output filename provided?
			if (outputFile != NULL)
			{
				path = outputFile;
				path.pathNormalize();
				name = path;
				path.pathDirname();
				name.pathBasename();
			}
			else
			{
				path = filename;
				path.pathDirname();
				name = filename;
				name.pathBasename();
			}
			
			int extensionIndex = hkString::lastIndexOf(name, '.');
			if (extensionIndex >= 0)
				name.slice(0, extensionIndex);

			converter.saveScenes(path, name);
		}
		else
		{
			HK_ERROR(0x0, "Failed to convert the scene!\n");
			fbxSdkManager->Destroy();
			return -1;
		}

		fbxSdkManager->Destroy();
	}

	// quit Havok
	{
		hkBaseSystem::quit();
		hkMemoryInitUtil::quit();
	}
	
	return 0;
}

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
