

#![feature(map_first_last)]

use crate::agentstatemap::AgentStateMap;

use crate::influencemap::InfluenceMap;

use crate::connectionset::ConnectionSet;

use crate::statetype::StateType;

use crate::influencetype::InfluenceType;

use crate::pathlist::PathList;


use std::collections::HashMap;



fn main() {
    
    
    
}



fn calculatepath(agentstate: AgentStateMap, startingnode: u32, connectionmap: ConnectionSet, connections: HashMap<u32, InfluenceMap> ){
    
    //create a pathlist
    let mut pathlist = PathList::new();
    
    pathlist.add_first_path(agentstate.clone(), startingnode);
    
    
    for x in 0..10000{
        
        if let Some( (prevpathid, prevstate ) ) = pathlist.pop_highest_path(){
            
            //get the nodeid that this pathid ends on
            let mut prevnodeid = pathlist.get_node_of_path(&prevpathid);
            
            //get th
            let mut currentsegments = connectionmap.get_connected_segment_ids( prevnodeid );
            

            //for each segment
            for (currentsegmentid, nextnodeid) in currentsegments{
                
                //get the influence for this segment
                let currentinfluence = connections.get(&currentsegmentid).unwrap();
                
                //create a new agent state cloned from the old one
                let mut newagentstate = agentstate.clone();
                
                currentinfluence.apply_to_state(&mut newagentstate);

                pathlist.add_path(prevpathid, newagentstate, nextnodeid);
                
            }

            
        }
        else
        {
            break;
        }
        
        
        
    }
    
    
    
}


mod statetype{
    
    #[derive(Clone, Debug, PartialEq, Eq, Hash)]
    pub enum StateType{
        
        Timetaken,
        Health,
        Mana,
        Speed,
        DamageTo(u32),
        HealthOf(u32),
        NonSpecificGoal,
        
        
    }
}

mod influencetype{
    
    #[derive(Debug, PartialEq, Eq, Hash)]
    pub enum InfluenceType{
        
        Distance,
        NonSpecificGoal
        
        
    }
}

//an influence that affects an agentstatemap
mod influencemap{
    
    //the map of the influences
    //this is whats stored in the 
    use std::collections::HashMap;
    use crate::influencetype::InfluenceType;
    use crate::agentstatemap::AgentStateMap;
    use crate::statetype::StateType;
    
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
    
    
}

//an agent state map, what the state of the agent is
mod agentstatemap{
    
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
    
    
}




//holds the connections between nodeids and segmentids
mod connectionset{
    
    use std::collections::HashMap;
    use std::collections::HashSet;
    
    pub struct ConnectionSet{

        //the node to the segments it can take from it, and the node it ends on
        nodetosegment: HashMap<u32, HashSet< (u32,u32) > >,


    }
    
    impl ConnectionSet{
        
        pub fn new() -> ConnectionSet{
            
            ConnectionSet{ nodetosegment: HashMap::new() }
            
        }
        
        //get the ids of the segments connected to this node
        pub fn get_connected_segment_ids(&self, nodeid: u32) -> HashSet<(u32, u32)>{
            
            if let Some(somehashset) = self.nodetosegment.get(&nodeid){
                
                somehashset.clone()
                
            }
            //if this nodeid doesnt have a map to attached segments, its assumed to have none
            else{
                HashSet::new()
            }
            
        }
        
        
        
        
        
    }
    
    
    
}



//holds all the paths
//and by whether theyve been popped yet
//and sorted by score
mod pathlist{
    
    
    use ordered_float::NotNan;
    
    
    use crate::heapwithvalue::HeapWithValue;
    
    
    use std::collections::HashMap;
    use std::collections::HashSet;
    
    use crate::influencemap::InfluenceMap;
    use crate::influencetype::InfluenceType;
    use crate::agentstatemap::AgentStateMap;
    use crate::statetype::StateType;
    
    
    
    pub struct PathList{
        
        //the total number of paths ever created, to give each path a unique ID
        totalpaths: u32,
        
        //a list of the paths to their agentstatemap
        pathtoagentstate: HashMap<u32, AgentStateMap>,
        
        
        //a list of the unpopped paths sorted by their score
        unpoppedsortedpaths: HeapWithValue,
        
        
        //for every path that has an originator, has its path id as a key in this map
        //and the value is to the pathid that is the path it comes from
        pathtooriginator: HashMap<u32, u32>,
        
        
        //the nodeid each path is on
        pathtonode: HashMap<u32, u32>,
        
    }
    
    /*
    
    functions:
    
    add path (previous path, statemap)
    pop highest value unpopped path
    get the list of paths this path has taken to get here    
    
    */
    
    impl PathList{
        
        pub fn new() -> PathList{
            
            PathList{
                totalpaths: 0,
                pathtoagentstate: HashMap::new(),
                unpoppedsortedpaths: HeapWithValue::new(),
                pathtooriginator: HashMap::new(),
                pathtonode: HashMap::new()
            }
        }

        pub fn get_node_of_path(&self, pathid: &u32) -> u32{

            *self.pathtonode.get(pathid).unwrap()

        }
        
        
        pub fn add_first_path(&mut self, agentstatemap: AgentStateMap, nodeid: u32){
            
            let pathid = self.totalpaths;
            self.totalpaths += 1;
            
            let thispathscore = agentstatemap.get_state_score();
            
            self.unpoppedsortedpaths.insert(thispathscore, pathid);
            self.pathtoagentstate.insert(pathid, agentstatemap);
            //self.pathtooriginator.insert(pathid, previouspathid);
            self.pathtonode.insert(pathid, nodeid);
            
            
        }
        
        
        
        pub fn add_path(&mut self, previouspathid: u32, agentstatemap: AgentStateMap, nodeid: u32){
            
            let pathid = self.totalpaths;
            self.totalpaths += 1;
            
            let thispathscore = agentstatemap.get_state_score();
            
            self.unpoppedsortedpaths.insert(thispathscore, pathid);
            self.pathtoagentstate.insert(pathid, agentstatemap);
            self.pathtooriginator.insert(pathid, previouspathid);
            self.pathtonode.insert(pathid, nodeid);
            
            
        }
        
        //get the highest value path and make it so it cant be gotten again
        //return an option of the pathid and its agentstatemap
        pub fn pop_highest_path(&mut self) -> Option<(u32, AgentStateMap)>{
            
            
            if let Some( (pathvalue, pathid) ) = self.unpoppedsortedpaths.pop(){
                
                let agentstatemap = self.pathtoagentstate.get(&pathid).unwrap();
                
                return(  Some( (pathid,agentstatemap.clone()) )  );
                
            }
            else{
                
                return(None);
            }
            
            
            
        }
        
        
        
    }
    
    
    
    
}



//a struct that you can store f32 that maps to u32
//and then pop to get the u32 with the highest f32 that hasnt been popped yet
mod heapwithvalue{
    
    
    use ordered_float::NotNan;
    use std::collections::BTreeMap;
    
    use std::collections::HashMap;
    use std::collections::HashSet;
    
    
    
    
    //this is a heap that has associated with it
    pub struct HeapWithValue{
        
        //the list of the values and the nodes
        mainlist: BTreeMap< NotNan<f32> , HashSet<u32> >,
        
    }
    
    
    impl HeapWithValue{
        
        pub fn new() -> HeapWithValue{
            
            HeapWithValue{ mainlist: BTreeMap::new()}
            
        }
        
        //get the id with the highest value and remove it from the list
        pub fn pop(&mut self) -> Option<( f32 , u32 )>{
            
            //if the list is empty, return None
            if (self.mainlist.is_empty()){
                
                return(None);
            }
            else{
                
                //get the id and the value
                let (lastkey, _) = self.mainlist.last_key_value().unwrap();
                
                let lastkey = lastkey.clone();
                let idset = self.mainlist.get_mut(&lastkey).unwrap();
                
                
                let value = lastkey.into_inner();
                
                
                let mut maybereturnid : Option<u32> = None;
                
                if (idset.len() == 0){
                    
                    panic!("WHAT IS GOING ON, why is there a key to an empty set of paths");
                }
                
                //iterate through just to grab and remove one value from the nodeset
                for curvalue in idset.iter(){
                    
                    maybereturnid = Some(*curvalue);
                    
                    break;
                }
                
                if let Some(returnid) = maybereturnid{            
                    
                    idset.remove(&returnid);
                    
                    if (idset.len() == 0){
                        self.mainlist.remove(&lastkey);
                    }
                    
                    
                    
                    //return the pathvalue and the id of the node
                    return( Some((value, returnid)) ); 
                    
                }
                else{
                    
                    panic!("the return node id didnt exist in thsi set of nodes");
                }
                
                
            }
            
            
            
            
            
        }
        
        
        
        //insert a u32 with its associated value
        pub fn insert( &mut self, value: f32 , id: u32 ){
            
            let key = NotNan::new( value ).expect("nan error");
            
            //check if it doesnt exist, make it first
            if (! self.mainlist.contains_key(&key)){
                
                self.mainlist.insert(key, HashSet::new());
            }
            
            
            self.mainlist.get_mut(&key).unwrap().insert(id);
            
        }
        
        
        
    }
}
