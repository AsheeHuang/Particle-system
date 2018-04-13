#include <cmath>
#include <iostream>
#include "configFile.h"
#include "CMassSpringSystem.h"
#include "glut.h"
#include "Render_API.h"

#pragma comment( lib, "glut32.lib" )

const double g_cdDeltaT = 0.001f;
const double g_cdK	   = 2500.0f;
const double g_cdD	   = 50.0f;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Constructor & Destructor
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
CMassSpringSystem::CMassSpringSystem()
   :m_bDrawParticle(true),
    m_bDrawStruct(false),
    m_bDrawShear(false),
    m_bDrawBending(false),
    m_bSimulation(false),

    m_iIntegratorType(EXPLICIT_EULER),

    m_dDeltaT(g_cdDeltaT),
    m_dSpringCoefStruct(g_cdK),
    m_dSpringCoefShear(g_cdK),
    m_dSpringCoefBending(g_cdK),
    m_dDamperCoefStruct(g_cdD),
    m_dDamperCoefShear(g_cdD),
    m_dDamperCoefBending(g_cdD),
    m_dRotate(0.0f),

    m_Offset(Vector3d::ZERO),
    m_ForceField(Vector3d(0.0,-9.8,0.0)),

    m_GoalNet(),
    m_Balls()
{
}

CMassSpringSystem::CMassSpringSystem(const std::string &a_rcsConfigFilename)
:m_GoalNet(a_rcsConfigFilename)
{
    int iIntegratorType;
    double dSpringCoef,dDamperCoef,dHeightOffset;

    ConfigFile configFile;
    configFile.suppressWarnings(1);

    configFile.addOption("DrawParticle"        ,&m_bDrawParticle);
    configFile.addOption("DrawSpringStructural",&m_bDrawStruct);
    configFile.addOption("DrawSpringShear"     ,&m_bDrawShear);
    configFile.addOption("DrawSpringBending"   ,&m_bDrawBending);
    configFile.addOption("SimulationStart"     ,&m_bSimulation);

    configFile.addOption("IntegratorType",&iIntegratorType);
    configFile.addOption("DeltaT",&m_dDeltaT);
    configFile.addOption("SpringCoef",&dSpringCoef);
    configFile.addOption("DamperCoef",&dDamperCoef);
    configFile.addOption("Rotate",&m_dRotate);
    configFile.addOption("HeightOffset",&dHeightOffset);

    int code = configFile.parseOptions((char *)a_rcsConfigFilename.c_str());
    if(code == 1)
    {
        std::cout<<"Error in CMassSpringSystem constructor."<<std::endl;
        system("pause");
        exit(0);
    }
    m_iIntegratorType = CMassSpringSystem::EXPLICIT_EULER;
    if(iIntegratorType == 1)
    {
        m_iIntegratorType = CMassSpringSystem::RUNGE_KUTTA;
    }

    m_dSpringCoefStruct  = dSpringCoef;
    m_dSpringCoefShear   = dSpringCoef;
    m_dSpringCoefBending = dSpringCoef;
    m_dDamperCoefStruct  = dDamperCoef;
    m_dDamperCoefShear   = dDamperCoef;
    m_dDamperCoefBending = dDamperCoef;

    if(dHeightOffset<0.0 || dHeightOffset>10.0)
        dHeightOffset = 0.0;

    m_Offset       = Vector3d(0.0,dHeightOffset,0.0);
    m_ForceField   = Vector3d(0.0,-9.8,0.0);

    Reset();
}

CMassSpringSystem::CMassSpringSystem(const CMassSpringSystem &a_rcMassSpringSystem)
    :m_bDrawParticle(a_rcMassSpringSystem.m_bDrawParticle),
    m_bDrawStruct(a_rcMassSpringSystem.m_bDrawStruct),
    m_bDrawShear(a_rcMassSpringSystem.m_bDrawShear),
    m_bDrawBending(a_rcMassSpringSystem.m_bDrawBending),
    m_bSimulation(a_rcMassSpringSystem.m_bSimulation),

    m_iIntegratorType(a_rcMassSpringSystem.m_iIntegratorType),

    m_dDeltaT(a_rcMassSpringSystem.m_dDeltaT),
    m_dSpringCoefStruct(a_rcMassSpringSystem.m_dSpringCoefStruct),
    m_dSpringCoefShear(a_rcMassSpringSystem.m_dSpringCoefShear),
    m_dSpringCoefBending(a_rcMassSpringSystem.m_dSpringCoefBending),
    m_dDamperCoefStruct(a_rcMassSpringSystem.m_dDamperCoefStruct),
    m_dDamperCoefShear(a_rcMassSpringSystem.m_dDamperCoefShear),
    m_dDamperCoefBending(a_rcMassSpringSystem.m_dDamperCoefBending),
    m_dRotate(a_rcMassSpringSystem.m_dRotate),

    m_Offset(a_rcMassSpringSystem.m_Offset),
    m_ForceField(a_rcMassSpringSystem.m_ForceField)
{
}
CMassSpringSystem::~CMassSpringSystem()
{
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Draw
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CMassSpringSystem::Draw()
{
    DrawGoalNet();
    DrawBall();
}

void CMassSpringSystem::DrawGoalNet()
{    
    // draw particle
    if (m_bDrawParticle)
    {
        glPushAttrib(GL_CURRENT_BIT);
        for (int uiI = 0; uiI < m_GoalNet.ParticleNum(); uiI++)
        {
            setColor3f(1.0, 0.0, 0.0);
            drawPoint(m_GoalNet.GetParticle(uiI).GetPosition(), 3.0);
        }
        glPopAttrib();
    }

    // draw spring
    glPushAttrib(GL_CURRENT_BIT);
    for (int uiI = 0; uiI < m_GoalNet.SpringNum(); uiI++)
    {
        if ((m_GoalNet.GetSpring(uiI).GetSpringType() == CSpring::Type_nStruct && m_bDrawStruct) ||
            (m_GoalNet.GetSpring(uiI).GetSpringType() == CSpring::Type_nShear && m_bDrawShear) ||
            (m_GoalNet.GetSpring(uiI).GetSpringType() == CSpring::Type_nBending && m_bDrawBending))
        {
            int iSpringStartID = m_GoalNet.GetSpring(uiI).GetSpringStartID();
            int iSpringEndID = m_GoalNet.GetSpring(uiI).GetSpringEndID();
            Vector3d springColor = m_GoalNet.GetSpring(uiI).GetSpringColor();
            Vector3d startPos = m_GoalNet.GetParticle(iSpringStartID).GetPosition();
            Vector3d endPos = m_GoalNet.GetParticle(iSpringEndID).GetPosition();

            setColor3fv(springColor);
            drawLine(startPos, endPos);                   
        }
    }
    glPopAttrib();

    // draw cylinder
    int widthNum = m_GoalNet.GetWidthNum();
    int heightNum = m_GoalNet.GetHeightNum();
    int lengthNum = m_GoalNet.GetLengthNum();

    int backBottomRightId = m_GoalNet.GetParticleID(0, 0, 0);
    int backBottomLeftId = m_GoalNet.GetParticleID(0, 0, lengthNum-1);
    int frontBottomRightId = m_GoalNet.GetParticleID(widthNum-1, 0, 0);
    int frontBottomLeftId = m_GoalNet.GetParticleID(widthNum-1, 0, lengthNum-1);
    int backTopRightId = m_GoalNet.GetParticleID(0, heightNum-1, 0);
    int backTopLeftId = m_GoalNet.GetParticleID(0, heightNum-1, lengthNum-1);
    int frontTopRightId = m_GoalNet.GetParticleID(widthNum-1, heightNum-1, 0);
    int frontTopLeftId = m_GoalNet.GetParticleID(widthNum-1, heightNum-1, lengthNum-1);

	// for vollyball baseket
	Vector3d touchFloor(0, -0.5, 0);
    /*
    drawCylinder(
        m_GoalNet.GetParticle(backBottomRightId).GetPosition(), 
        m_GoalNet.GetParticle(backBottomLeftId).GetPosition(), 
        0.05);
    drawCylinder(
        m_GoalNet.GetParticle(backBottomRightId).GetPosition(),
        m_GoalNet.GetParticle(frontBottomRightId).GetPosition(),
        0.05);
    drawCylinder(
        m_GoalNet.GetParticle(backBottomLeftId).GetPosition(),
        m_GoalNet.GetParticle(frontBottomLeftId).GetPosition(),
        0.05);
    */
    drawCylinder(
		m_GoalNet.GetParticle(backBottomLeftId).GetPosition() + touchFloor,
        m_GoalNet.GetParticle(backTopLeftId).GetPosition(),
        0.05);
    drawCylinder(
		m_GoalNet.GetParticle(backBottomRightId).GetPosition() + touchFloor,
        m_GoalNet.GetParticle(backTopRightId).GetPosition(),
        0.05);
    drawCylinder(
        m_GoalNet.GetParticle(backTopRightId).GetPosition(),
        m_GoalNet.GetParticle(backTopLeftId).GetPosition(),
        0.05);
    drawCylinder(
        m_GoalNet.GetParticle(backTopRightId).GetPosition(),
        m_GoalNet.GetParticle(frontTopRightId).GetPosition(),
        0.05);
    drawCylinder(
        m_GoalNet.GetParticle(backTopLeftId).GetPosition(),
        m_GoalNet.GetParticle(frontTopLeftId).GetPosition(),
        0.05);
    drawCylinder(
        m_GoalNet.GetParticle(frontTopRightId).GetPosition(),
        m_GoalNet.GetParticle(frontTopLeftId).GetPosition(),
        0.05);
	drawCylinder(
		m_GoalNet.GetParticle(frontBottomRightId).GetPosition() + touchFloor,
        m_GoalNet.GetParticle(frontTopRightId).GetPosition(),
        0.05);
    drawCylinder(
		m_GoalNet.GetParticle(frontBottomLeftId).GetPosition() + touchFloor,
        m_GoalNet.GetParticle(frontTopLeftId).GetPosition(),
        0.05);
}

void CMassSpringSystem::DrawBall()
{
    for (unsigned int ballIdx = 0; ballIdx < BallNum(); ++ballIdx)
    {
        drawSolidBall(m_Balls[ballIdx].GetPosition(), m_Balls[ballIdx].GetRadius());
    }
}


Vector3d CMassSpringSystem::CalcTriangleNormal(CParticle &p1,CParticle &p2,CParticle &p3)
{
    Vector3d pos1 = p1.GetPosition();
    Vector3d pos2 = p2.GetPosition();
    Vector3d pos3 = p3.GetPosition();

    Vector3d v1 = pos2-pos1;
    Vector3d v2 = pos3-pos1;

    return v1.CrossProduct(v2);
}

void CMassSpringSystem::DrawShadow(const Vector3d &a_rLightPos)
{
}
void CMassSpringSystem::DrawShadowPolygon(const Vector3d &a_rLightPos,const Vector3d &a_rNormalVec,
                                          const Vector3d &a_rVerPos1,const Vector3d &a_rVerPos2)
{
    Vector3d ShadowPos1,ShadowPos2,LightVec;
    LightVec = (a_rVerPos1 - a_rLightPos);
    LightVec.Normalize();
    ShadowPos1 = a_rVerPos1 + (a_rVerPos1 - a_rLightPos) * 5.0f;
    ShadowPos2 = a_rVerPos2 + (a_rVerPos2 - a_rLightPos) * 5.0f;
    
    if(a_rNormalVec.DotProduct(LightVec)<=0.0f)
    {
        glBegin( GL_QUADS );
        glVertex3dv( a_rVerPos1.val );
        glVertex3dv( ShadowPos1.val );
        glVertex3dv( ShadowPos2.val );
        glVertex3dv( a_rVerPos2.val );
        glEnd();
    }
    else
    {
        glBegin( GL_QUADS );
        glVertex3dv( a_rVerPos1.val );
        glVertex3dv( a_rVerPos2.val );
        glVertex3dv( ShadowPos2.val );
        glVertex3dv( ShadowPos1.val );
        glEnd();
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Set and Update
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CMassSpringSystem::Reset()
{ 
    m_GoalNet.Reset();
    m_Balls.clear();
}

void CMassSpringSystem::SetSpringCoef(const double a_cdSpringCoef, const CSpring::enType_t a_cSpringType)
{
    if (a_cSpringType == CSpring::Type_nStruct)
    {
        m_dSpringCoefStruct = a_cdSpringCoef;
        m_GoalNet.SetSpringCoef(a_cdSpringCoef, CSpring::Type_nStruct);
    }
    else if (a_cSpringType == CSpring::Type_nShear)
    {
        m_dSpringCoefShear = a_cdSpringCoef;
        m_GoalNet.SetSpringCoef(a_cdSpringCoef, CSpring::Type_nShear);
    }
    else if (a_cSpringType == CSpring::Type_nBending)
    {
        m_dSpringCoefBending = a_cdSpringCoef;
        m_GoalNet.SetSpringCoef(a_cdSpringCoef, CSpring::Type_nBending);
    }
    else
    {
        std::cout << "Error spring type in CMassSpringSystme SetSpringCoef" << std::endl;
    }

}

void CMassSpringSystem::SetDamperCoef(const double a_cdDamperCoef, const CSpring::enType_t a_cSpringType)
{
    if (a_cSpringType == CSpring::Type_nStruct)
    {
        m_dDamperCoefStruct = a_cdDamperCoef;
        m_GoalNet.SetDamperCoef(a_cdDamperCoef, CSpring::Type_nStruct);
    }
    else if (a_cSpringType == CSpring::Type_nShear)
    {
        m_dDamperCoefShear = a_cdDamperCoef;
        m_GoalNet.SetDamperCoef(a_cdDamperCoef, CSpring::Type_nShear);
    }
    else if (a_cSpringType == CSpring::Type_nBending)
    {
        m_dDamperCoefBending = a_cdDamperCoef;
        m_GoalNet.SetDamperCoef(a_cdDamperCoef, CSpring::Type_nBending);
    }
    else
    {
        std::cout << "Error spring type in CMassSpringSystme SetDamperCoef" << std::endl;
    }
}

double CMassSpringSystem::GetSpringCoef(const CSpring::enType_t a_cSpringType)
{
    if(a_cSpringType == CSpring::Type_nStruct)
    {
        return m_dSpringCoefStruct;
    }
    else if(a_cSpringType == CSpring::Type_nShear)
    {
        return m_dSpringCoefShear;
    }
    else if(a_cSpringType == CSpring::Type_nBending)
    {
        return m_dSpringCoefBending;
    }
    else
    {
        std::cout<<"Error spring type in CMassSpringSystme GetSpringCoef"<<std::endl;
        return -1.0;
    }
}
double CMassSpringSystem::GetDamperCoef(const CSpring::enType_t a_cSpringType)
{
    if(a_cSpringType == CSpring::Type_nStruct)
    {
        return m_dDamperCoefStruct;
    }
    else if(a_cSpringType == CSpring::Type_nShear)
    {
        return m_dDamperCoefShear;
    }
    else if(a_cSpringType == CSpring::Type_nBending)
    {
        return m_dDamperCoefBending;
    }
    else
    {
        std::cout<<"Error spring type in CMassSpringSystme GetDamperCoef"<<std::endl;
        return -1.0;
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Simulation Part
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool CMassSpringSystem::CheckStable()
{
    double threshold = 1e6;
    for(int pIdx = 0; pIdx < m_GoalNet.ParticleNum(); pIdx++)
    {
        Vector3d particleVel = m_GoalNet.GetParticle(pIdx).GetVelocity();
        if (particleVel.Magnitude() > threshold)
        {
            return false;
        }  
    }
    return true;
}
void CMassSpringSystem::SimulationOneTimeStep()
{
    if(m_bSimulation)
    {
	    //ComputeAllForce();
        Integrate();
    }
    
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Initialization
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void CMassSpringSystem::CreateBall()
{
    // randomly assign initial velocity and position to a ball
    Ball newBall;
	//Vector3d randomOffset((double)(rand() % 5 + 5.0), (double)(rand() % 5+5.0), (double)(rand() % 5));
	Vector3d randomOffset((double)(rand() % 5 + 5.0), (double)(rand() % 5 + 8.0), (double)(rand() % 5));
	Vector3d initBallPos = m_GoalNet.GetInitPos() + randomOffset;
    Vector3d initBallVel = (m_GoalNet.GetInitPos()- 2*initBallPos)*2.0;
    //Vector3d initBallVel = Vector3d::ZERO;
    newBall.SetPosition(initBallPos);
    newBall.SetVelocity(initBallVel);
    m_Balls.push_back(newBall);
}

int CMassSpringSystem::BallNum()
{
    return m_Balls.size();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Compute Force
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CMassSpringSystem::ResetAllForce()
{
    for (int pIdx = 0; pIdx < m_GoalNet.ParticleNum(); ++pIdx)
    {
        //TO DO 6
        m_GoalNet.GetParticle(pIdx).SetAcceleration(Vector3d::ZERO);
    }

    for (int ballIdx = 0; ballIdx < BallNum(); ++ballIdx)
    {
        m_Balls[ballIdx].SetAcceleration(Vector3d::ZERO);
    }
}

void CMassSpringSystem::ComputeAllForce()
{
    ComputeParticleForce();
    ComputeBallForce();
}

void CMassSpringSystem::ComputeParticleForce()
{
    m_GoalNet.AddForceField(m_ForceField);
    m_GoalNet.ComputeInternalForce();
}

void CMassSpringSystem::ComputeBallForce()
{
    for (int ballIdx = 0; ballIdx < BallNum(); ++ballIdx)
    {
        //m_Balls[ballIdx].SetAcceleration(m_ForceField);
        m_Balls[ballIdx].AddForce(m_ForceField * m_Balls[ballIdx].GetMass());
    }
}

void CMassSpringSystem::HandleCollision()
{
    NetPlaneCollision();
    BallPlaneCollision();
    BallToBallCollision();
    BallNetCollision();
}

void CMassSpringSystem::NetPlaneCollision()
{
    //TO DO 2
    static const double eEPSILON = 0.01;
    double resistCoef = 0.5;
    double frictionCoef = 0.3;
	Vector3d N = Vector3d::UNIT_Y;

	for (int pIdx = 0; pIdx < m_GoalNet.ParticleNum(); pIdx++){
		//detect collision
		CParticle* p = &m_GoalNet.GetParticle(pIdx);
		Vector3d pos = p->GetPosition();

		Vector3d project_p = Vector3d(pos.x, -0.98, pos.z); //project point on the plane
		double dist = (pos - project_p).DotProduct(N);

		if (dist < eEPSILON && N.DotProduct(p->GetVelocity())< 0){
				Vector3d f_c,f_f;
				if (N.DotProduct(p->GetForce()) < 0){
					Vector3d v_n = N.DotProduct(p->GetVelocity()) * N;
					Vector3d v_t = p->GetVelocity() - v_n;
					f_c = -(N.DotProduct(p->GetForce()))*N;
					f_f = -(frictionCoef * (-N.DotProduct(p->GetForce()))*p->GetVelocity());

					p->AddForce(f_f);
					p->SetVelocity(-resistCoef * v_n + v_t);
				}
		}
	}
}

void CMassSpringSystem::BallPlaneCollision()
{
    //TO DO 2
    static const double eEPSILON = 0.01;
    double resistCoef = 0.5;
    double frictionCoef = 0.3;

	Vector3d N = Vector3d::UNIT_Y;

	for (int ballIdx = 0; ballIdx < BallNum(); ballIdx++){
		//detect collision
		Ball* b = &m_Balls[ballIdx];
		Vector3d pos = b->GetPosition();

		Vector3d project_p = Vector3d(pos.x, -1, pos.z); //project point on the plane
		double dist = (pos - project_p).DotProduct(N) - b->GetRadius();

		if (dist < eEPSILON && N.DotProduct(b->GetVelocity())<eEPSILON ){
			if (N.DotProduct(b->GetForce()) < 0){
				Vector3d v, f_f;
				Vector3d v_n = N.DotProduct(b->GetVelocity()) * N;
				Vector3d v_t = b->GetVelocity() - v_n;
				v =- resistCoef * v_n + v_t;
				f_f = -(frictionCoef * (-N.DotProduct(b->GetForce()))*b->GetVelocity());

				b->AddForce(f_f );
				b->SetVelocity(v);
			}
		}
	}
   
}

void CMassSpringSystem::BallToBallCollision()
{
    static const double eEPSILON = 0.01;
	//TO DO 2
	for (int b1Idx = 0; b1Idx < BallNum(); b1Idx++){
		Ball* b1 = &m_Balls[b1Idx];
		Vector3d b1_pos = b1->GetPosition();
		for (int b2Idx = 0; b2Idx < BallNum(); b2Idx++){
			if (b1Idx != b2Idx){
				Ball* b2 = &m_Balls[b2Idx];
				Vector3d b2_pos = b2->GetPosition();
				if ((b1_pos - b2_pos).Length() - b1->GetRadius() - b2->GetRadius() < eEPSILON){
					Vector3d v1 = b1->GetVelocity();
					Vector3d v2 = b2->GetVelocity();
					Vector3d v1_n = v1.DotProduct(b1_pos - b2_pos) * (b1_pos - b2_pos);
					Vector3d v2_n = v2.DotProduct(b2_pos - b1_pos) * (b2_pos - b1_pos);
					Vector3d v1_t = v1 - v1_n;
					Vector3d v2_t = v2 - v2_n;
					double m1 = b1->GetMass();
					double m2 = b2->GetMass();

					b1->SetVelocity((v1_n*(m1 - m2) + 2 * m2*v2_n) / (m1 + m2) + v1_t);
					b2->SetVelocity((v2_n*(m2 - m1) + 2 * m1*v1_n) / (m1 + m2) + v2_t);
				}
			}
		}
	}
}
bool isInTheNet(Vector3d pos){
	double x = pos.x;
	double y = pos.y;
	double z = pos.z;

	if (x > 0 && x < 3 && z > 0 && z < 3)
		return true;
	else
		return false;
}
void CMassSpringSystem::BallNetCollision()
{
    static const double eEPSILON = 0.01;
	
	for (int b1Idx = 0 ; b1Idx < BallNum(); b1Idx++){
		Ball* b = &m_Balls[b1Idx];
		Vector3d b_pos = b->GetPosition();
	
		for (int pIdx = 0; pIdx < m_GoalNet.ParticleNum(); pIdx++){
			CParticle* p = &m_GoalNet.GetParticle(pIdx);
			Vector3d p_pos = p->GetPosition();
			Vector3d N = p->GetNormal();
			if (isInTheNet(b_pos) == false)
				N = -N;

			if ((b_pos - p_pos).Length() - b->GetRadius() < eEPSILON && N.DotProduct(b->GetVelocity()) < 0){
				Vector3d v1 = p->GetVelocity();
				Vector3d v2 = b->GetVelocity();
				Vector3d v1_n = v1.DotProduct(p_pos - b_pos) * (p_pos - b_pos);
				Vector3d v2_n = v2.DotProduct(N) * N;
				Vector3d v1_t = v1 - v1_n;
				Vector3d v2_t = v2 - v2_n;
				double m1 = p->GetMass();
				double m2 = b->GetMass();

				p->SetVelocity((v1_n*(m1 - m2) + 2 * m2 * v2_n) / (m1 + m2) + v1_t);
				b->SetVelocity(-0.5*v2_n + 0.3*v2_t);
			}
		}
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Integrator
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CMassSpringSystem::Integrate()
{
    ResetContactForce();

    if(m_iIntegratorType == CMassSpringSystem::EXPLICIT_EULER)
    {
        ComputeAllForce();
        HandleCollision();
	    ExplicitEuler();
        ResetAllForce();
    }
    else if(m_iIntegratorType == CMassSpringSystem::RUNGE_KUTTA)
    {
		ComputeAllForce();
		HandleCollision();
        RungeKutta();
        ResetAllForce();
    }
    else
    {
        std::cout<<"Error integrator type, use explicit Euler instead!!"<<std::endl;
        ComputeAllForce();
        HandleCollision();
        ExplicitEuler();
        ResetAllForce();
    }
}

void CMassSpringSystem::ExplicitEuler()
{
    // Goal Net
    for (int pIdx = 0; pIdx < m_GoalNet.ParticleNum(); ++pIdx)
    {
		CParticle* p = &m_GoalNet.GetParticle(pIdx);
		p->AddVelocity(m_dDeltaT * p->GetAcceleration());
		p->AddPosition( m_dDeltaT * p->GetVelocity());
    }

    // Balls
    for (int ballIdx = 0; ballIdx < BallNum(); ++ballIdx)
    {
		//TO DO 6
		Ball* b = &m_Balls[ballIdx];
		b->AddVelocity(m_dDeltaT * b->GetAcceleration());
		b->AddPosition(m_dDeltaT * b->GetVelocity());
    }
}

void CMassSpringSystem::RungeKutta()
{
    //TO DO 7
    ParticleRungeKutta();
    BallRungeKutta();
}

void CMassSpringSystem::ParticleRungeKutta()
{
	//TO DO 7
    struct StateStep
    {
        Vector3d deltaVel;
        Vector3d deltaPos;
		StateStep(Vector3d deltaPos, Vector3d deltaVel){
			this->deltaPos = deltaPos;
			this->deltaVel = deltaVel;
		}
    };

    //container to store k1~k4 for each particles
    vector<Vector3d> curPosCntr, curVelCntr;
    vector<StateStep> k1StepCntr, k2StepCntr, k3StepCntr, k4StepCntr;
	
	for (int pIdx = 0; pIdx < m_GoalNet.ParticleNum(); pIdx++){
		CParticle* p = &m_GoalNet.GetParticle(pIdx);
		//cout << p->GetForce() << endl;
		//p->AddVelocity(m_dDeltaT * p->GetAcceleration());
		Vector3d pos = p->GetPosition();
		Vector3d vel = p->GetVelocity();
		Vector3d acc = p->GetAcceleration();
		curPosCntr.push_back(pos);
		curVelCntr.push_back(vel);

		//k1
		CParticle p1 = m_GoalNet.GetParticle(pIdx);
		Vector3d k1_deltaPos = m_dDeltaT * vel;
		Vector3d k1_deltaVel = m_dDeltaT * acc;
		StateStep k1_s = StateStep(k1_deltaPos,k1_deltaVel);
		p1.AddPosition(k1_deltaPos);
		p1.AddVelocity(k1_deltaVel);
		k1StepCntr.push_back(k1_s);
		//k2
		CParticle p2 = m_GoalNet.GetParticle(pIdx);
		Vector3d vel2 = p1.GetVelocity();
		Vector3d acc2 = p1.GetAcceleration();

		Vector3d k2_deltaPos = vel2 / 2 * m_dDeltaT;
		Vector3d k2_deltaVel = acc2 / 2 * m_dDeltaT;
		StateStep k2_s = StateStep(k2_deltaPos, k2_deltaVel);
		p2.AddPosition(k2_deltaPos);
		p2.AddVelocity(k2_deltaVel);
		k2StepCntr.push_back(k2_s);
		//k3
		CParticle p3 = m_GoalNet.GetParticle(pIdx);
		Vector3d vel3 = p2.GetVelocity();
		Vector3d acc3 = p2.GetAcceleration();

		Vector3d k3_deltaPos = vel3 / 2 * m_dDeltaT;
		Vector3d k3_deltaVel = acc3 / 2 * m_dDeltaT;
		StateStep k3_s = StateStep(k3_deltaPos, k3_deltaVel);
		p3.AddPosition(k3_deltaPos);
		p3.AddVelocity(k3_deltaVel);
		k3StepCntr.push_back(k3_s);
		//k4
		CParticle p4 = m_GoalNet.GetParticle(pIdx);
		Vector3d vel4 = p3.GetVelocity();
		Vector3d acc4 = p3.GetAcceleration();

		Vector3d k4_deltaPos = vel4 * m_dDeltaT ;
		Vector3d k4_deltaVel = acc4 * m_dDeltaT;
		StateStep k4_s = StateStep(k4_deltaPos, k4_deltaVel);
		k4StepCntr.push_back(k4_s);
		p4.AddPosition(k4_deltaPos);
		p4.AddVelocity(k4_deltaVel);

		//CParticle* p = &m_GoalNet.GetParticle(pIdx);
		
	}
	for (int pIdx = 0; pIdx < m_GoalNet.ParticleNum(); pIdx++){
		CParticle* p = &m_GoalNet.GetParticle(pIdx);
		Vector3d k1_p = k1StepCntr[pIdx].deltaPos;
		Vector3d k2_p = k2StepCntr[pIdx].deltaPos;
		Vector3d k3_p = k3StepCntr[pIdx].deltaPos;
		Vector3d k4_p = k4StepCntr[pIdx].deltaPos;
		//cout << k1_p << " " << k2_p << " " << k3_p << " " << k4_p << endl;
		Vector3d delta_p = (k1_p + 2 * k2_p + 2 * k3_p + k4_p)/4 ;
		p->AddPosition(delta_p);
		//cout << delta_p.x << " " << delta_p.y << " " << delta_p.z << endl;
		Vector3d k1_v = k1StepCntr[pIdx].deltaVel;
		Vector3d k2_v = k2StepCntr[pIdx].deltaVel;
		Vector3d k3_v = k3StepCntr[pIdx].deltaVel;
		Vector3d k4_v = k4StepCntr[pIdx].deltaVel;
		Vector3d delta_v = (k1_v + 2 * k2_v + 2 * k3_v + k4_v)/4;
		//cout << delta_v.x << " " << delta_v.y << " " << delta_v.z << endl;
		p->AddVelocity(delta_v);
	}

}

void CMassSpringSystem::BallRungeKutta()
{
	//TO DO 7
    struct StateStep
    {
        Vector3d deltaVel;
        Vector3d deltaPos;
		StateStep(Vector3d deltaPos, Vector3d deltaVel){
			this->deltaPos = deltaPos;
			this->deltaVel = deltaVel;
		}
    };

    //container to store k1~k4 for each particles
    vector<Vector3d> curPosCntr, curVelCntr;
    vector<StateStep> k1StepCntr, k2StepCntr, k3StepCntr, k4StepCntr;

	for (int pIdx = 0; pIdx < BallNum(); pIdx++){
		Ball * p = &m_Balls[pIdx];

		//p->AddVelocity(m_dDeltaT * p->GetAcceleration());
		Vector3d pos = p->GetPosition();
		Vector3d vel = p->GetVelocity();
		Vector3d acc = p->GetAcceleration();
		curPosCntr.push_back(pos);
		curVelCntr.push_back(vel);

		//k1
		Ball p1 = m_Balls[pIdx];
		Vector3d k1_deltaPos = m_dDeltaT * vel;
		Vector3d k1_deltaVel = m_dDeltaT * acc;
		StateStep k1_s = StateStep(k1_deltaPos, k1_deltaVel);
		p1.AddPosition(k1_deltaPos);
		p1.AddVelocity(k1_deltaVel);
		k1StepCntr.push_back(k1_s);
		//k2
		Ball p2 = m_Balls[pIdx];
		Vector3d vel2 = p1.GetVelocity();
		Vector3d acc2 = p1.GetAcceleration();

		Vector3d k2_deltaPos = vel2 / 2 * m_dDeltaT;
		Vector3d k2_deltaVel = acc2 / 2 * m_dDeltaT;
		StateStep k2_s = StateStep(k2_deltaPos, k2_deltaVel);
		p2.AddPosition(k2_deltaPos);
		p2.AddVelocity(k2_deltaVel);
		k2StepCntr.push_back(k2_s);
		//k3
		Ball p3 = m_Balls[pIdx];
		Vector3d vel3 = p2.GetVelocity();
		Vector3d acc3 = p2.GetAcceleration();

		Vector3d k3_deltaPos = vel3 / 2 * m_dDeltaT;
		Vector3d k3_deltaVel = acc3 / 2 * m_dDeltaT;
		StateStep k3_s = StateStep(k3_deltaPos, k3_deltaVel);
		p3.AddPosition(k3_deltaPos);
		p3.AddVelocity(k3_deltaVel);
		k3StepCntr.push_back(k3_s);
		//k4
		Ball p4 = m_Balls[pIdx];
		Vector3d vel4 = p3.GetVelocity();
		Vector3d acc4 = p3.GetAcceleration();

		Vector3d k4_deltaPos = vel4 * m_dDeltaT;
		Vector3d k4_deltaVel = acc4 * m_dDeltaT;
		StateStep k4_s = StateStep(k4_deltaPos, k4_deltaVel);
		k4StepCntr.push_back(k4_s);
		p4.AddPosition(k4_deltaPos);
		p4.AddVelocity(k4_deltaVel);


	}
	for (int pIdx = 0; pIdx < BallNum(); pIdx++){
		Ball * p = &m_Balls[pIdx];
		Vector3d k1_p = k1StepCntr[pIdx].deltaPos;
		Vector3d k2_p = k2StepCntr[pIdx].deltaPos;
		Vector3d k3_p = k3StepCntr[pIdx].deltaPos;
		Vector3d k4_p = k4StepCntr[pIdx].deltaPos;
		Vector3d delta_p = (k1_p + 2 * k2_p + 2 * k3_p + k4_p)/4;
		p->AddPosition(delta_p);
		//cout << delta_p.x << " " << delta_p.y << " " << delta_p.z << endl;
		Vector3d k1_v = k1StepCntr[pIdx].deltaVel;
		Vector3d k2_v = k2StepCntr[pIdx].deltaVel;
		Vector3d k3_v = k3StepCntr[pIdx].deltaVel;
		Vector3d k4_v = k4StepCntr[pIdx].deltaVel;
		Vector3d delta_v = (k1_v + 2 * k2_v + 2 * k3_v + k4_v)/4;
		//cout << delta_v.x << " " << delta_v.y << " " << delta_v.z << endl;
		p->AddVelocity(delta_v);
	}

}

void CMassSpringSystem::ResetContactForce()
{
    m_BallContactForce.clear();
    m_BallContactForce.resize(BallNum());
    for (int ballIdx = 0; ballIdx < BallNum(); ++ballIdx)
    {
        m_BallContactForce.push_back(Vector3d::ZERO);
    }
    m_ParticleContactForce.clear();
    m_ParticleContactForce.resize(m_GoalNet.ParticleNum());
    for (int pIdx = 0; pIdx < m_GoalNet.ParticleNum(); ++pIdx)
    {
        m_ParticleContactForce.push_back(Vector3d::ZERO);
    }
}