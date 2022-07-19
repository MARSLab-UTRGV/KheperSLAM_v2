/* Include the controller definition */
#include "kheper_diffusion_lidar.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

/****************************************/
/****************************************/

CKheperDiffusionLidar::CKheperDiffusionLidar() :
   m_pcWheels(NULL),
  m_pcOdometry(NULL),
  m_pcLidar(NULL),
  m_cSLAMData("kheper_data_1.dat", std::ofstream::out | std::ofstream::trunc),
  m_cTestData("test_data.dat", std::ofstream::out | std::ofstream::trunc),
   m_pcProximity(NULL),
   m_cAlpha(10.0f),
   m_fDelta(0.5f),
   m_fWheelVelocity(2.5f),
   m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
                           ToRadians(m_cAlpha)) {}

/****************************************/
/****************************************/

void CKheperDiffusionLidar::Init(TConfigurationNode& t_node) {
   /*
    * Get sensor/actuator handles
    *
    * The passed string (ex. "differential_steering") corresponds to the
    * XML tag of the device whose handle we want to have. For a list of
    * allowed values, type at the command prompt:
    *
    * $ argos3 -q actuators
    *
    * to have a list of all the possible actuators, or
    *
    * $ argos3 -q sensors
    *
    * to have a list of all the possible sensors.
    *
    * NOTE: ARGoS creates and initializes actuators and sensors
    * internally, on the basis of the lists provided the configuration
    * file at the <controllers><footbot_diffusion><actuators> and
    * <controllers><footbot_diffusion><sensors> sections. If you forgot to
    * list a device in the XML and then you request it here, an error
    * occurs.
    */
   m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcProximity = GetSensor  <CCI_KheperaIVProximitySensor      >("kheperaiv_proximity"    );

    m_pcOdometry  = GetSensor  <CCI_DifferentialSteeringSensor  >("differential_steering");
    m_pcLidar     = GetSensor  <CCI_KheperaIVLIDARSensor        >("kheperaiv_lidar"      );
   /*
    * Parse the configuration file
    *
    * The user defines this part. Here, the algorithm accepts three
    * parameters and it's nice to put them in the config file so we don't
    * have to recompile if we want to try other settings.
    */
   GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
   m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
   GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
   GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
    //m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
}

/****************************************/
/****************************************/

void CKheperDiffusionLidar::ControlStep() {

  /* get odom readings */
  const CCI_DifferentialSteeringSensor::SReading& tOdomReads = m_pcOdometry->GetReading();
  m_leftOdom += tOdomReads.CoveredDistanceLeftWheel;//*147.4*10.0;
  m_rightOdom += tOdomReads.CoveredDistanceRightWheel;//*147.4*10.0;

  std::size_t numLidReads = m_pcLidar->GetNumReadings();
  //const CCI_KheperaIVLIDARSensor::TReadings& tLidarNumReads = m_pcLidar->GetNumReadings();

  m_unTicks++;
  /* record data */
  if(m_unTicks%1 == 0){
    /* test file */
    //m_cTestData << "left odom " << m_leftOdom << ", right odom " << m_rightOdom;
    //m_cTestData << ", lidar count " << numLidReads << "\n";
    m_cSLAMData << (UInt64)(m_unTicks*100*1000) << " ";
    m_cSLAMData << 0 << " ";
    //odom
    m_cSLAMData << (UInt64)(m_leftOdom) << " "<< (UInt64)(m_rightOdom) << " ";
    // more unused data
    for(size_t j = 0; j < 20; ++j ){
       m_cSLAMData << 0 << " " ;
    }
    for(std::size_t i = 0; i < numLidReads; ++i){
    m_cSLAMData << i << " " << (UInt32)m_pcLidar->GetReading(i)*10;
    }
    m_cSLAMData << "\n";
/*
    ///* slam data (using offline_slam.cpp as reference)
    m_cSLAMData << (UInt64)(m_unTicks*100*1000) << " ";
    //unused data
    m_cSLAMData << 0 << " ";
    //odom
    m_cSLAMData << (UInt64)(m_leftOdom) << " "<< (UInt64)(m_rightOdom) << " ";
    // more unused data
    for(size_t j = 0; j < 20; ++j ){
       m_cSLAMData << 0 << " " ;
    }

    // lidar
    for(std::size_t i = 0; i < numLidReads; ++i){
      m_cSLAMData << m_pcLidar->GetReading(i) << " " ;
    }
    m_cSLAMData << "\n";
*/

  }


   /* Get readings from proximity sensor */
   const CCI_KheperaIVProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
   /* Sum them together */
   CVector2 cAccumulator;
   for(size_t i = 0; i < tProxReads.size(); ++i) {
      cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
   }
   cAccumulator /= tProxReads.size();
   /* If the angle of the vector is small enough and the closest obstacle
    * is far enough, continue going straight, otherwise curve a little
    */
   CRadians cAngle = cAccumulator.Angle();
   if(m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
      cAccumulator.Length() < m_fDelta ) {
      /* Go straight */
      m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
   }
   else {
      /* Turn, depending on the sign of the angle */
      if(cAngle.GetValue() > 0.0f) {
         m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
      }
      else {
         m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
      }
   }
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CKheperDiffusionLidar, "kheper_diffusion_lidar_controller")
