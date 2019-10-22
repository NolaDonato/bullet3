
INCLUDE_DIRECTORIES(
	${BULLET_PHYSICS_SOURCE_DIR}/examples/Importers/ImportURDFDemo
	${BULLET_PHYSICS_SOURCE_DIR}/examples/Importers/ImportOBJDemo
	${BULLET_PHYSICS_SOURCE_DIR}/examples/Importers/ImportColladaDemo
	${BULLET_PHYSICS_SOURCE_DIR}/examples/Importers/ImportMeshUtility
	${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs
	${BULLET_PHYSICS_SOURCE_DIR}/examples/CommonInterfaces
	${BULLET_PHYSICS_SOURCE_DIR}/src
)

ADD_LIBRARY(URDFConverterLib
	${BULLET_PHYSICS_SOURCE_DIR}/Extras/Serialize/BulletWorldImporter/btWorldImporter.cpp
	${BULLET_PHYSICS_SOURCE_DIR}/Extras/Serialize/BulletWorldImporter/btMultiBodyWorldImporter.cpp
	${BULLET_PHYSICS_SOURCE_DIR}/Extras/Serialize/BulletWorldImporter/btBulletWorldImporter.cpp
	${BULLET_PHYSICS_SOURCE_DIR}/Extras/Serialize/BulletFileLoader/bChunk.cpp		
	${BULLET_PHYSICS_SOURCE_DIR}/Extras/Serialize/BulletFileLoader/bFile.cpp
	${BULLET_PHYSICS_SOURCE_DIR}/Extras/Serialize/BulletFileLoader/bDNA.cpp		
	${BULLET_PHYSICS_SOURCE_DIR}/Extras/Serialize/BulletFileLoader/btBulletFile.cpp	
	${BULLET_PHYSICS_SOURCE_DIR}/examples/Importers/ImportObjDemo/LoadMeshFromObj.cpp
	${BULLET_PHYSICS_SOURCE_DIR}/examples/Importers/ImportColladaDemo/LoadMeshFromCollada.cpp
	${BULLET_PHYSICS_SOURCE_DIR}/examples/Importers/ImportMeshUtility/b3ImportMeshUtility.cpp
    ${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/Wavefront/tiny_obj_loader.cpp
    ${BULLET_PHYSICS_SOURCE_DIR}/examples/Importers/ImportObjDemo/Wavefront2GLInstanceGraphicsShape.cpp
	${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/stb_image/stb_image.cpp
	${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/stb_image/stb_image_write.cpp
)

IF (BUILD_SHARED_LIBS)
	IF (WIN32)
                TARGET_LINK_LIBRARIES(URDFConverterLib
                        Bullet3Common BulletDynamics BulletCollision BulletInverseDynamicsUtils
                        BulletInverseDynamics LinearMath
                )
	ELSE(WIN32)
                IF(APPLE)
                 TARGET_LINK_LIBRARIES(URDFConverterLib
                                Bullet3Common BulletDynamics BulletCollision BulletInverseDynamicsUtils
                                BulletInverseDynamics LinearMath
                 )
  		ELSE(APPLE)
                 TARGET_LINK_LIBRARIES(URDFConverterLib
                                Bullet3Common BulletDynamics BulletCollision BulletInverseDynamicsUtils
                                BulletInverseDynamics LinearMath
                        )
        ENDIF(APPLE)
	ENDIF(WIN32)
ENDIF(BUILD_SHARED_LIBS)

INSTALL(TARGETS URDFConverterLib
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib${LIB_SUFFIX}
        ARCHIVE DESTINATION lib${LIB_SUFFIX})


INCLUDE_DIRECTORIES(
    ${BULLET_PHYSICS_SOURCE_DIR}/src
	${BULLET_PHYSICS_SOURCE_DIR}/examples/SharedMemory
	${BULLET_PHYSICS_SOURCE_DIR}/examples/Importers/ImportURDFDemo
    ${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs
	${BULLET_PHYSICS_SOURCE_DIR}/examples/CommonInterfaces
)


LINK_LIBRARIES(
        URDFConverterLib Bullet3Common BulletSoftBody BulletDynamics BulletCollision
        BulletInverseDynamicsUtils BulletInverseDynamics LinearMath
)

SET(URDFConverter_Sources
	${BULLET_PHYSICS_SOURCE_DIR}/examples/Importers/ImportURDFDemo/URDFConverter.h
	${BULLET_PHYSICS_SOURCE_DIR}/examples/Importers/ImportURDFDemo/URDFConverter.cpp
	${BULLET_PHYSICS_SOURCE_DIR}/examples/Importers/ImportURDFDemo/URDF2Bullet.h
        ${BULLET_PHYSICS_SOURCE_DIR}/examples/Importers/ImportURDFDemo/URDF2Bullet.cpp
        ${BULLET_PHYSICS_SOURCE_DIR}/examples/Importers/ImportURDFDemo/UrdfParser.cpp
        ${BULLET_PHYSICS_SOURCE_DIR}/examples/Importers/ImportURDFDemo/urdfStringSplit.cpp
        ${BULLET_PHYSICS_SOURCE_DIR}/examples/Importers/ImportURDFDemo/urdfStringSplit.h
        ${BULLET_PHYSICS_SOURCE_DIR}/examples/Importers/ImportURDFDemo/BulletUrdfImporter.cpp
        ${BULLET_PHYSICS_SOURCE_DIR}/examples/Importers/ImportURDFDemo/BulletUrdfImporter.h
	${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/tinyxml2/tinyxml2.cpp
	${BULLET_PHYSICS_SOURCE_DIR}/build3/bullet.rc
)

ADD_EXECUTABLE(App_URDFConverter ${URDFConverter_Sources})


IF (BULLET2_USE_TBB_MULTITHREADING AND WIN32)
    # add a post build command to copy some dlls to the executable directory
    set(TBB_VC_VER "vc12")
    set(TBB_VC_ARCH "ia32")
    # assume 32-bit build in VC12 for now
    # checks can be added here at a later time
    ADD_CUSTOM_COMMAND(TARGET App_URDFConverter POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${BULLET2_TBB_INCLUDE_DIR}/../bin/${TBB_VC_ARCH}/${TBB_VC_VER}/tbb.dll"
        $<TARGET_FILE_DIR:App_URDFConverter>)
    ADD_CUSTOM_COMMAND(TARGET App_URDFConverter POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${BULLET2_TBB_INCLUDE_DIR}/../bin/${TBB_VC_ARCH}/${TBB_VC_VER}/tbbmalloc.dll"
        $<TARGET_FILE_DIR:App_URDFConverter>)
ENDIF (BULLET2_USE_TBB_MULTITHREADING AND WIN32)


IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
			SET_TARGET_PROPERTIES(App_ExampleBrowser PROPERTIES  DEBUG_POSTFIX "_Debug")
			SET_TARGET_PROPERTIES(App_ExampleBrowser PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
			SET_TARGET_PROPERTIES(App_ExampleBrowser PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)

