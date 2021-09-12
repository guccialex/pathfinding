
use nalgebra::Point2;
use ncollide2d::shape::Segment;
use ncollide2d::partitioning::DBVTLeafId;
use std::collections::HashMap;

use crate::influencemap::InfluenceMap;



#[derive(Debug)]
pub struct PathfindingSegment{
    
    //the id of the two nodes this segment is between
    node1id: u32,
    node2id: u32,
    
    segment: Segment<f32>,
    
    leafid: DBVTLeafId,
    
    influencemap1to2: InfluenceMap,
    
    influencemap2to1: InfluenceMap,
    
}

impl PathfindingSegment{
    
    pub fn new(thesegment: Segment<f32>, dbvtleafid: DBVTLeafId, node1id: u32, node2id: u32, mut evalmap: HashMap<u32, f32>) -> PathfindingSegment{
        
        //get the length of the segment
        let segmentlength = thesegment.length();
        
        //multiple the evalmap by the length of the segment
        scaleevalmap(&mut evalmap, segmentlength);
        
        
        PathfindingSegment{
            segment: thesegment,
            leafid: dbvtleafid,
            node1id: node1id,
            node2id: node2id,
            influencemap1to2: evalmap.clone(),
            influencemap2to1: evalmap
        }
    }
    
    //create a new pathfinding segment where the direction of the path matters
    pub fn newdirectional(thesegment: Segment<f32>, dbvtleafid: DBVTLeafId, node1id: u32, node2id: u32, evalmap1to2: HashMap<u32, f32>, evalmap2to1: HashMap<u32, f32>) -> PathfindingSegment{
        
        PathfindingSegment{
            segment: thesegment,
            leafid: dbvtleafid,
            node1id: node1id,
            node2id: node2id,
            evalmap1to2: evalmap1to2,
            evalmap2to1: evalmap2to1
        }
    }
    
    //given a node to start at, get the influence map to get to the other end, and the node it ends at
    pub fn getothernodeandvalue(&self, thisnodeid: &u32) -> (u32, f32){
        
        if (*thisnodeid == self.node1id){
            (self.node2id, getevaluation(&self.evalmap1to2, agentevalmap ) )
            
        }
        else if ( *thisnodeid == self.node2id ) {
            (self.node1id, getevaluation(&self.evalmap2to1, agentevalmap ) )
        }
        else{
            panic!("this segment doesnt have this nodeid at all, why?");
        }
        
    }
    
    
    //return a tuple of the nodeids in an arbitrary order
    pub fn getnodes(&self) -> (u32, u32){
        
        (self.node1id, self.node2id)
    }
    
    ///get the leafid that corresponds to this segments leaf on the segmentdbvt
    pub fn getleafid(&self) -> DBVTLeafId{
        
        self.leafid
        
    }
    
    //get the ncollide2d segment that represents this segments shape
    pub fn getshapesegment(&self) -> Segment<f32>{
        self.segment.clone()
    }
    
    
    //get the points this segments shape ends at
    pub fn getendpoints(&self) -> (Point2<f32> , Point2<f32>){
        
        (*self.segment.a(), *self.segment.b())
        
    }
    
    
}



fn scaleevalmap(evalmap: &mut HashMap<u32, f32>, scalefactor: f32) {
    
    
    //for each of the values in evalmap2
    for (id, value) in evalmap.iter_mut(){
        
        *value = *value * scalefactor;
        
    }
    
}