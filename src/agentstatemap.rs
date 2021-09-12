
use crate::statetype::StateType;
use std::collections::HashMap;


#[derive(Clone)]
pub struct AgentStateMap{
    
    pub mainmap: HashMap<StateType, f32>,
    
}

impl AgentStateMap{
    
    pub fn new() -> AgentStateMap{
        
        AgentStateMap{mainmap: HashMap::new()}
        
    }
    
    //get a value that doesnt have multiple values
    pub fn get_value(&self, statetype: &StateType) -> f32{
        
        
        let value = self.mainmap.get(statetype).unwrap();
        
        
        *value
        
    }
    
    pub fn add_to_value(&mut self, statetype: &StateType, value: f32){
        
        let mut statevalue = self.mainmap.get_mut(statetype).unwrap();
        
        *statevalue += value;
        
        
    }
    
    
    pub fn get_state_score(&self) -> f32{
        
        let mut returnscore = 0.0;
        
        //how much to multiply the end returnscore by
        let mut multfactor = 1.0;
        
        
        for (statetype, value) in self.mainmap.iter(){
            
            if statetype == &StateType::Timetaken{
                
                multfactor = multfactor / value;
                
            }
            else if statetype == &StateType::Health{
                
                returnscore += value;
                
            }
            else if statetype == &StateType::Mana{
                
                returnscore += value;
                
            }
            else if let StateType::DamageTo(id) = statetype{
                
                returnscore += value;
                
            }
            else{
                panic!("uncaught statetype {:?}", statetype);
            }
            
            
            
        }
        
        
        returnscore * multfactor
        
        
    }
    
}