
//the map of the influences
//this is whats stored in the 
use std::collections::HashMap;
use crate::influencetype::InfluenceType;
use crate::agentstatemap::AgentStateMap;
use crate::statetype::StateType;


#[derive(Debug)]
pub struct InfluenceMap{
    
    pub mainmap: HashMap< InfluenceType ,f32 >,
    
}

impl InfluenceMap{
    
    pub fn new() -> InfluenceMap{
        
        InfluenceMap{mainmap: HashMap::new()}
        
    }
    
    //apply this influence map to an AgentState
    pub fn apply_to_state(&self, agentstatemap: &mut AgentStateMap){
        
        
        
        for (influencetype, value) in self.mainmap.iter(){
            
            
            if influencetype == &InfluenceType::Distance{
                
                let speedvalue = agentstatemap.get_value( &StateType::Speed);
                
                let deltatime = value / speedvalue;
                
                agentstatemap.add_to_value( &StateType::Speed, deltatime);
                
            }        
        }
        
        
    }
    
    
}


