/* Include the controller definition */
#include "footbot_diffusion.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

/****************************************/
/****************************************/

CFootBotDiffusion::CFootBotDiffusion() :
   m_pcWheels(NULL),
   m_pcProximity(NULL),
   m_cAlpha(10.0f),
   m_fDelta(0.5f),
   m_fWheelVelocity(2.5f),
   m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
                           ToRadians(m_cAlpha)) {}

/****************************************/
/****************************************/

void CFootBotDiffusion::Init(TConfigurationNode& t_node) {
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
   m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );
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
}

/****************************************/
/****************************************/

int CFootBotDiffusion::obstacle_decide_motion_direction (const CCI_FootBotProximitySensor::TReadings& tProxReads){
	int s, l, r, rev;
	s = l = r = rev = 0;
	// straight path is clear
	if (!(tProxReads[0].Value || tProxReads[21].Value || tProxReads[1].Value || tProxReads[22].Value || tProxReads[2].Value || tProxReads[23].Value))
		s = 1;
	
	// left path is clear
	if (!(tProxReads[3].Value || tProxReads[8].Value || tProxReads[4].Value || tProxReads[7].Value || tProxReads[5].Value || tProxReads[6].Value))
		l = 1;

	//right path is clear
	if (!(tProxReads[15].Value || tProxReads[20].Value || tProxReads[16].Value || tProxReads[19].Value || tProxReads[17].Value || tProxReads[18].Value))
		r = 1;

	// reverse can be taken
	if(!(tProxReads[12].Value || tProxReads[11].Value || tProxReads[13].Value)) 
		rev = 1;

	if (s){									// straight path can be taken
		if(!l && !r)	return 3;			// take reverse if left and right are blocked
		else if(l && !r)	return 1;		// turn left if left is free, right is blocked
		else if(!l && r)	return 2;		// turn right if left if blocked, right is free
		else	return 0;					// go straight if both left and and right are free
   }  
	
	else{									// obstacle ahead, straight cannot be taken
		if(!l && !r)	return 3;			// take reverse if both left and and right are blocked
		else if(l && !r)	return 1;		// turn left if left is free, right is blocked
		else if(!l && r)	return 2;		// turn right if left if blocked, right is free
		else	return 1;					// turn left if both left and right are free
   }
}

void CFootBotDiffusion::ControlStep() {
   /* Get readings from proximity sensor */
   const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
   /* Sum them together */
   CVector2 cAccumulator;
   for(size_t i = 0; i < tProxReads.size(); ++i) {
      cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
   }
   cAccumulator /= tProxReads.size();

   is_obs = CFootBotDiffusion::obstacle_decide_motion_direction(tProxReads);
   switch (is_obs){
	case 0:{	// go straight
		m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
		break;
	}
	case 1:{	// turn left
		m_pcWheels->SetLinearVelocity(1.0f, m_fWheelVelocity);
		break;
	}
	case 2:{	// turn right
		m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 1.0f);
		break;
	}
	case 3:{	// take reverse
		m_pcWheels->SetLinearVelocity(-1 * m_fWheelVelocity, -1 * m_fWheelVelocity);
		break;
	}
	default:{	// go straight
      	m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
	   	break;
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
REGISTER_CONTROLLER(CFootBotDiffusion, "footbot_diffusion_controller")
