#include "foraging_qt_user_functions.h"
#include <controllers/kheper_foraging/kheper_foraging.h>
#include <argos3/core/simulator/entity/controllable_entity.h>

using namespace argos;

/****************************************/
/****************************************/

CForagingQTUserFunctions::CForagingQTUserFunctions() {
   RegisterUserFunction<CForagingQTUserFunctions,CFootBotEntity>(&CForagingQTUserFunctions::Draw);
}

/****************************************/
/****************************************/

void CForagingQTUserFunctions::Draw(CKheperaIVEntity& c_entity) {
   CKheperaIVEntity& cController = dynamic_cast<CKheperaIVForaging&>(c_entity.GetControllableEntity().GetController());
   CKheperaIVForaging::SFoodData& sFoodData = cController.GetFoodData();
   if(sFoodData.HasFoodItem) {
      DrawCylinder(
         CVector3(0.0f, 0.0f, 0.3f), 
         CQuaternion(),
         0.1f,
         0.05f,
         CColor::BLACK);
   }
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CKheperForagingQTUserFunctions, "kheper_foraging_qt_user_functions")
