#ifndef URDF_CONVERTER_H
#define URDF_CONVERTER_H

#include <string>
#include "CommonMultiBodyBase.h"
#include "MultiBodyCreationInterface.h"

class btMultiBodyDynamicsWorld;
class btRigidBody;
class btTransform;
class btVector3;
class btCollisionObject;
class btMultiBodyLinkCollider;
class btGeneric6DofSpring2Constraint;


class URDFConverter : public CommonMultiBodyBase
{
    std::string m_urdfFileName;
    bool m_useMultiBody;
    btMultiBodyDynamicsWorld* m_dynamicsWorld;

public:
    URDFConverter(bool isMultiBody);
   ~URDFConverter();

    btMultiBodyDynamicsWorld* importPhysics(const char* urdfFileName);
    bool exportPhysics(const char* bulletFileName);
    void initPhysics();
private:
    btMultiBodyDynamicsWorld* createWorld();
};

class MultiBodyCreator : public MultiBodyCreationInterface
{
    btMultiBody* m_bulletMultiBody;
    btRigidBody* m_rigidBody;
    btAlignedObjectArray<btGeneric6DofSpring2Constraint*> m_6DofConstraints;

public:
    btRigidBody* allocateRigidBody(int urdfLinkIndex, btScalar mass,
                                   const btVector3& localInertiaDiagonal,
                                   const btTransform& initialWorldTrans,
                                   class btCollisionShape* colShape);
    btMultiBody* allocateMultiBody(int urdfLinkIndex, int totalNumJoints,
                                   btScalar mass, const btVector3& localInertiaDiagonal,
                                   bool isFixedBase, bool canSleep);
    btMultiBodyLinkCollider* allocateMultiBodyLinkCollider(int /*urdfLinkIndex*/,
                                                           int mbLinkIndex, btMultiBody* multiBody);
    btGeneric6DofSpring2Constraint* allocateGeneric6DofSpring2Constraint(int urdfLinkIndex, 
                                                                         btRigidBody& rbA /*parent*/, 
                                                                         btRigidBody& rbB,
                                                                         const btTransform& offsetInA,
                                                                         const btTransform& offsetInB, 
                                                                         int rotateOrder = 0);
    btGeneric6DofSpring2Constraint* createPrismaticJoint(int urdfLinkIndex, 
                                                         btRigidBody& rbA /*parent*/, 
                                                         btRigidBody& rbB, 
                                                         const btTransform& offsetInA, 
                                                         const btTransform& offsetInB,
                                                         const btVector3& jointAxisInJointSpace, 
                                                         btScalar jointLowerLimit, btScalar jointUpperLimit);
    btGeneric6DofSpring2Constraint* createRevoluteJoint(int urdfLinkIndex,
                                                        btRigidBody& rbA /*parent*/,
                                                        btRigidBody& rbB,
                                                        const btTransform& offsetInA,
                                                        const btTransform& offsetInB,
                                                        const btVector3& jointAxisInJointSpace,
                                                        btScalar jointLowerLimit, btScalar jointUpperLimit);
    btGeneric6DofSpring2Constraint* createFixedJoint(int urdfLinkIndex,
                                                     btRigidBody& rbA /*parent*/, 
                                                     btRigidBody& rbB, 
                                                     const btTransform& offsetInA,
                                                     const btTransform& offsetInB);
    void addLinkMapping(int urdfLinkIndex, int mbLinkIndex);
    void createRigidBodyGraphicsInstance(int linkIndex, btRigidBody* body, const btVector3& colorRgba, int graphicsIndex)  { }

    void createRigidBodyGraphicsInstance2(int linkIndex, class btRigidBody* body, 
                                          const btVector3& colorRgba, const btVector3& specularColor, int graphicsIndex)  { }

    void createCollisionObjectGraphicsInstance(int linkIndex, class btCollisionObject* colObj, const btVector3& colorRgba)  { }

    void createCollisionObjectGraphicsInstance2(int linkIndex, class btCollisionObject* col,
                                                const btVector4& colorRgba, const btVector3& specularColor)  { }

    btMultiBody* getBulletMultiBody();

    btAlignedObjectArray<int> m_mb2urdfLink;
};
#endif  //URDF_CONVERTER_H
