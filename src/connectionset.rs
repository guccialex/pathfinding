use std::collections::HashMap;
use std::collections::HashSet;

pub struct ConnectionSet{
    
    //each segment is connected to two nodes
    segmenttonode: HashMap<u32, (u32, u32)>,
    
    //each node has a set of segments its connected to
    nodetosegment: HashMap<u32, HashSet<u32>>,
}

impl ConnectionSet{
    
    pub fn new() -> ConnectionSet{
        
        ConnectionSet{segmenttonode: HashMap::new(), nodetosegment: HashMap::new()}
        
    }
    
    //get the ids of the segments connected to this node
    pub fn get_connected_segment_ids(&self, nodeid: u32) -> HashSet<u32>{
        
        if let Some(somehashset) = self.nodetosegment.get(&nodeid){
            
            somehashset.clone()
            
        }
        //if this nodeid doesnt have a map to attached segments, its assumed to have none
        else{
            HashSet::new()
        }
        
    }
    
    //remove this segment
    pub fn remove_segment(&mut self, segmentid: u32){
        
        if ( !self.segmenttonode.contains_key(&segmentid)){
            
            panic!("this segmentid isnt in this connection set");
        }
        
        let (node1id, node2id) = self.segmenttonode.remove(&segmentid).unwrap();
        
        self.nodetosegment.get_mut(&node1id).unwrap().remove(&segmentid);
        
        self.nodetosegment.get_mut(&node2id).unwrap().remove(&segmentid);
        
    }
    
    //removes the node with this id, panics if there are any segments attached to it
    pub fn remove_node(&mut self, nodeid:u32){
        
        if (self.nodetosegment.get(&nodeid).unwrap().len() != 0){
            
            panic!("this node still has segments attached, cant remove it");
            
        }
        else{
            
            self.nodetosegment.remove(&nodeid);
        }
        
    }
    
    pub fn add_segment(&mut self, node1id: u32, node2id: u32, segmentid: u32){
        
        //if the map for this node doesnt exist
        if ( ! self.nodetosegment.contains_key(&node1id) ){
            
            self.nodetosegment.insert(node1id, HashSet::new() );
            
        }
        
        //if the map for this node doesnt exist
        if ( ! self.nodetosegment.contains_key(&node2id) ){
            
            self.nodetosegment.insert(node2id, HashSet::new() );
            
        }
        
        
        self.segmenttonode.insert(segmentid, (node1id, node2id) );
        
        self.nodetosegment.get_mut(&node1id).unwrap().insert(segmentid);
        self.nodetosegment.get_mut(&node2id).unwrap().insert(segmentid);
        
    }
    
    
    
    
}


