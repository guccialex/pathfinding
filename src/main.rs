
#![feature(map_first_last)]

use std::collections::HashMap;
use std::collections::BTreeMap;

use ncollide2d::shape::Polyline;
use ncollide2d::shape::Segment;
use nalgebra::geometry::Point2;
use nalgebra::Isometry2;
use ncollide2d::shape::ConvexPolygon;
use nalgebra::Vector2;

use ncollide2d::shape::shape::Shape;

use std::collections::HashSet;

use ordered_float::NotNan;

use ncollide2d::partitioning::DBVTLeafId;

use ncollide2d::partitioning::BVT;
use ncollide2d::bounding_volume;
use ncollide2d::bounding_volume::AABB;
use ncollide2d::query::visitors::AABBSetsInterferencesCollector;
use nalgebra::base::Matrix2;
use ncollide2d::partitioning::BVH;
use ncollide2d::query;
use ncollide2d::shape::Ball;
use ncollide2d::bounding_volume::BoundingSphere;
use ncollide2d::query::visitors::BoundingVolumeInterferencesCollector;

use ncollide2d::shape::Compound;

use ncollide2d::shape::ShapeHandle;

use ncollide2d::partitioning::DBVT;

use ncollide2d::partitioning::DBVTLeaf;
use ncollide2d::bounding_volume::bounding_volume::BoundingVolume;

use ncollide2d::query::proximity;

use nalgebra::Unit;


use query::PointQuery;

use std::cmp::Ordering::Equal;

use std::collections::BTreeSet;


use ncollide2d::bounding_volume::bounding_volume::HasBoundingVolume;


fn main() {
    println!("Hello, world!");
    
    
    let mut mypathfinding = Pathfinding::new();
    
    
    
    
    
    
    let points = [
    Point2::new(-3.0, -2.0),
    Point2::new(-3.0, 1225.0),
    Point2::new(1225.0, 1225.0),
    Point2::new(1225.0, -2.0),
    ];
    
    let theshape = ConvexPolygon::try_from_points(&points).expect("Convex hull computation failed.");
    let theisometry = Isometry2::new(Vector2::new(50.0, 50.0), 0.0);//std::f32::consts::FRAC_PI_2);
    
    let mut pathmap = HashMap::new();
    pathmap.insert(1, 5.0);
    let mut entermap = HashMap::new();
    let mut exitmap = HashMap::new();
    
    
    let shape1 = mypathfinding.addshape( theshape, theisometry, pathmap.clone(), entermap, exitmap);
    
    
    
    
    let points = [
    Point2::new(200.0, 200.0),
    Point2::new(203.0, 925.0),
    Point2::new(925.0, 925.0),
    Point2::new(925.0, 202.0),
    ];
    
    let theshape = ConvexPolygon::try_from_points(&points).expect("Convex hull computation failed.");
    let theisometry = Isometry2::new(Vector2::new(50.0, 50.0), 0.0);//std::f32::consts::FRAC_PI_2);
    
    let mut pathmap = HashMap::new();
    pathmap.insert(1, 500.0);
    //let mut entermap = HashMap::new();
    //let mut exitmap = HashMap::new();
    
    
    //let shape1 = mypathfinding.addshape( theshape, theisometry, pathmap.clone(), entermap, exitmap);
    
    
    
    
    
    
    let mut evalmap = HashMap::new();
    evalmap.insert(1, 1.0);
    
    
    mypathfinding.getpath(10, evalmap);
    
    mypathfinding.drawstate();
    
    
    
}








//responsible for:
//holding the set of shapes each node is in
#[derive(Debug)]
struct ShapeSetManager{
    
    //for each set of shapes, the set of nodes that are inside it
    shapesets: HashMap< BTreeSet<u32>, HashSet<u32> >,
    
    //for each node, the set of shapes its in
    shapesetofnode: HashMap< u32, BTreeSet<u32> >,
    
}

impl  ShapeSetManager{
    
    //functions the shapeset manager needs
    //get the shapeset of each node
    //compare if two nodes are in the same shapeset
    //get the list of all the nodes which are in the same shapeset as another node
    
    fn new() ->ShapeSetManager{
        
        ShapeSetManager{shapesets: HashMap::new(), shapesetofnode: HashMap::new()}
    }
    
    //checks if this node is inside a certain shape according to this struct
    fn isinsideshape(&self, shapeid: u32, nodeid: u32) -> bool{
        
        //returns true if this is inside the shape, false otherwise
        self.shapesetofnode.get(&nodeid).unwrap().contains(&shapeid)
        
        
    }
    
    
    //get a nodeid with the set of shapes its in and add it to this object
    fn addnode(&mut self, nodeid: u32, shapeset: BTreeSet<u32>) {
        
        //associate the shapeset its in with this nodeid
        self.shapesetofnode.insert(nodeid, shapeset.clone());
        
        //for this set of shapes, add this node to the list of nodes in that set
        //if it doesnt exist, create it 
        if let Some(nodesofshapeset) = self.shapesets.get_mut(&shapeset){
            
            //nodesofshapeset.insert(nodeid);
        } 
        else{
            
            self.shapesets.insert(shapeset.clone(), HashSet::new() );
        }
        
        //and then add this node to it
        self.shapesets.get_mut(&shapeset).unwrap().insert(nodeid);
        
        
    }
    
    fn getshapesetofnode(&self, nodeid: &u32) -> BTreeSet<u32>{
        
        self.shapesetofnode.get(&nodeid).unwrap().clone()
        
    }
    
    //get the list of the OTHER nodes that are in the same shapeset as the node passed in
    fn getnodesinsameshapeset(&self, nodeid: &u32) -> HashSet<u32>{
        
        let theshapeset = self.shapesetofnode.get(&nodeid).unwrap();
        
        let mut setofnodestoreturn = self.shapesets.get(theshapeset).unwrap().clone();
        
        setofnodestoreturn.remove(&nodeid);
        
        setofnodestoreturn
        
        
    }
    
    
    fn updateshapeset(&mut self, nodeid:u32, shapeset: HashSet<u32>){
        
        
        let mut copyshapesetofnode = self.shapesetofnode.get(&nodeid).unwrap().clone();
        
        //update the old shapeset to get rid of this node
        let mut theoldshapeset = self.getormakeshapeset(&copyshapesetofnode);
        theoldshapeset.remove(&nodeid);
        
        
        //clear the shapeset of this node
        let mut theshapesetofnode = self.shapesetofnode.get_mut(&nodeid).unwrap();
        for curshapeid in shapeset{
            theshapesetofnode.insert(curshapeid);
        }        
        let theshapesetofnode = theshapesetofnode.clone();
        
        //update the new shapeset to add this node
        let mut thenewshapeset = self.getormakeshapeset(&theshapesetofnode);
        thenewshapeset.insert(nodeid);
        
    }
    
    
    //add a shape to the shapeset a node is in
    fn addtoshapeset(&mut self, nodeid: u32, shapeid: u32){
        
        let mut copyshapesetofnode = self.shapesetofnode.get(&nodeid).unwrap().clone();
        
        //update the old shapeset to get rid of this node
        let mut theoldshapeset = self.getormakeshapeset(&copyshapesetofnode);
        theoldshapeset.remove(&nodeid);
        
        
        //add to the shapeset of this node
        let mut theshapesetofnode = self.shapesetofnode.get_mut(&nodeid).unwrap();
        theshapesetofnode.insert(shapeid);
        let theshapesetofnode = theshapesetofnode.clone();
        
        //update the new shapeset to add this node
        let mut thenewshapeset = self.getormakeshapeset(&theshapesetofnode);
        thenewshapeset.insert(nodeid);
        
    }
    
    //returns the shapeset, if it doesnt exist, make it first
    fn getormakeshapeset(&mut self, shapeset: &BTreeSet<u32>) -> &mut HashSet< u32 >{
        
        
        if let Some(nodesofshapeset) = self.shapesets.get_mut(&shapeset){
        } 
        else{
            self.shapesets.insert(shapeset.clone(), HashSet::new() );        
        }
        
        return ( self.shapesets.get_mut(&shapeset).unwrap() );
        
        
    }
    
}



#[derive(Debug)]
pub struct PathfindingShape{
    
    //this shapes isometry
    pos: Isometry2<f32>,
    
    //this shapes convex polygon
    shape: ConvexPolygon<f32>,
    
    //map for this shape value to move through
    pathmap: HashMap<u32, f32>,
    
    entermap: HashMap<u32, f32>,
    
    exitmap: HashMap<u32, f32>,
    
    
    //the map of nodes inside this shape, and to their corresponding outside node
    insidetooutsidemap: HashMap<u32, u32>,
    
    //the map of nodes outside this shape, and to their corresponding inside node
    outsidetoinsidemap: HashMap<u32, u32>,
    
}

impl PathfindingShape{
    
    fn new(thepos: Isometry2<f32>, theshape: ConvexPolygon<f32>, pathmap: HashMap<u32, f32>, entermap: HashMap<u32, f32>, exitmap: HashMap<u32, f32> ) -> PathfindingShape{
        
        PathfindingShape{
            pos: thepos,
            shape: theshape,
            pathmap: pathmap,
            entermap: entermap,
            exitmap: exitmap,
            insidetooutsidemap: HashMap::new(),
            outsidetoinsidemap: HashMap::new()
        }
    }
    
    fn addinsideoutsidenodes(&mut self, insidenode: &u32, outsidenode: &u32){
        
        self.insidetooutsidemap.insert( *insidenode, *outsidenode);
        
        self.outsidetoinsidemap.insert( *outsidenode, *insidenode);
        
    }
    
    //return the node , as well as the costmap to go from this node to that node, and that node to this node
    fn getnodeandcostmap(&self, node1: &u32) -> (u32, HashMap<u32, f32>, HashMap<u32, f32> ){
        
        //if its in the insidetooutsidemap
        if let Some(node2) = self.insidetooutsidemap.get(&node1){
            
            return( (*node2, self.exitmap.clone(), self.entermap.clone() ) );
        }
        
        if let Some(node2) = self.outsidetoinsidemap.get(&node1){
            
            return( (*node2, self.entermap.clone(), self.exitmap.clone() ) );
        }
        
        panic!("this node isnt on this shape");
        
        
    }
    
}


#[derive(Debug)]
pub struct PathfindingSegment{
    
    //the id of the two nodes this segment is between
    node1id: u32,
    node2id: u32,
    
    segment: Segment<f32>,
    
    leafid: DBVTLeafId,
    
    evalmap1to2: HashMap<u32, f32>,
    
    evalmap2to1: HashMap<u32, f32>,
    
}

impl PathfindingSegment{
    
    fn new(thesegment: Segment<f32>, dbvtleafid: DBVTLeafId, node1id: u32, node2id: u32, evalmap: HashMap<u32, f32>) -> PathfindingSegment{
        
        PathfindingSegment{
            segment: thesegment,
            leafid: dbvtleafid,
            node1id: node1id,
            node2id: node2id,
            evalmap1to2: evalmap.clone(),
            evalmap2to1: evalmap
        }
    }
    
    //create a new pathfinding segment where the direction of the path matters
    fn newdirectional(thesegment: Segment<f32>, dbvtleafid: DBVTLeafId, node1id: u32, node2id: u32, evalmap1to2: HashMap<u32, f32>, evalmap2to1: HashMap<u32, f32>) -> PathfindingSegment{
        
        PathfindingSegment{
            segment: thesegment,
            leafid: dbvtleafid,
            node1id: node1id,
            node2id: node2id,
            evalmap1to2: evalmap1to2,
            evalmap2to1: evalmap2to1
        }
    }
    
    //given a node to start at, get the cost to get to the other end, and the node it ends at
    fn getothernodeandvalue(&self, thisnodeid: &u32, agentevalmap: &HashMap<u32, f32>) -> (u32, f32){
        
        if (*thisnodeid == self.node1id){
            (self.node2id, getevaluation(&self.evalmap1to2, agentevalmap ) )
            
        }
        else if ( *thisnodeid == self.node2id ) {
            (self.node1id, getevaluation(&self.evalmap2to1, agentevalmap ) )
        }
        else{
            panic!("this segment doesnt have this nodeid at all")
        }
        
    }
    
    //given a nodeid of one of
    fn getothernode(&self, thisnodeid: &u32) -> u32{
        
        if (*thisnodeid == self.node1id){
            
            self.node2id
            
        }
        else if ( *thisnodeid == self.node2id ) {
            
            self.node1id
            
        }
        else{
            panic!("this segment doesnt have this nodeid at all")
        }
        
        
    }
    
    
}



#[derive(Debug)]
pub struct PathfindingNode{
    
    
    //where the node actuall is, its actual point
    realpoint: Point2<f32>,
    
    //the point this node is on the edge of the shape its on
    basepoint: Point2<f32>,
    
    //the normal for the line this node is on
    //in the positive direction is going away from the shape/line
    //in the negative direction its going inside of the shape
    linenormal: Vector2<f32>,
    
    
    //a list of all ids of the segments that this node is on an end of
    attachedsegmentids: HashSet<u32>,
    
    
    //the shape it might be on
    optionshapeon: Option<u32>,
    
    
}

impl PathfindingNode{
    
    fn new( basepoint: Point2<f32>, linenormal: Vector2<f32> ) -> PathfindingNode{
        
        PathfindingNode{
            
            realpoint: basepoint,
            
            basepoint: basepoint,
            
            linenormal: linenormal,
            
            attachedsegmentids: HashSet::new(),
            
            optionshapeon: None,
            
        }
        
    }

    fn removeattachedsegment(&mut self, attachedsegmentid: &u32){
        self.attachedsegmentids.remove(attachedsegmentid);
    }
    
    fn newonshape( basepoint: Point2<f32>, linenormal: Vector2<f32>, shapetobeon: u32 ) -> PathfindingNode{
        
        PathfindingNode{
            
            realpoint: basepoint,
            
            basepoint: basepoint,
            
            linenormal: linenormal,
            
            attachedsegmentids: HashSet::new(),
            
            optionshapeon: Some(shapetobeon),
            
        }
        
    }
    
    
    //given a distance from the base, create a real node at that distance
    //and pass its position back
    fn createreal(&mut self, distancefrombase: f32) -> Point2<f32>{
        
        self.realpoint = self.basepoint + self.linenormal * distancefrombase;
        
        self.realpoint
        
    }
    
    fn addsegment(&mut self, segmentid: u32){
        
        self.attachedsegmentids.insert(segmentid);
        
    }
    
    //get a list of the segment ids of all the segments for this node
    fn get_segment_ids(&self) -> HashSet<u32>{
        
        self.attachedsegmentids.clone()
        
    }
    
    
}


struct Pathfinding{
    
    //how many shapes have ever existed, used to set pathfindingshapeid
    totalshapes: u32,
    //how many lines have ever existed, used to set id
    totallines: u32,
    //how many nodes have ever existed, used to set id
    totalnodes: u32,
    
    
    shapemap: HashMap<u32, PathfindingShape>,
    //other data associated with shapes:
    //the segments have a reference to the id of the nodes they're on
    //the shape in the dbvt
    //the shapeset manager has a reference to its ID

    
    segmentmap: HashMap<u32, PathfindingSegment>,
    //other data associated with segments:
    //both nodes have the reference to the segment that is between them
    //the shape in the dbvt


    nodemap: HashMap<u32, PathfindingNode>,
    //other data associated with nodes:
    //segments have a reference to teh nodes theyre on
    //shapes have reference to the id of the nodes on the
    //the shape in the dbvt
    //the shapeset manager has a reference for this ndoe


    shapedbvt: DBVT<f32, u32, AABB<f32>>,
    connectiondbvt: DBVT<f32, u32, AABB<f32>>,
    realnodedbvt: DBVT<f32, u32, AABB<f32>>,
    
    
    //the maximum amount of length there can be of a line without a node
    linelengthpernode: f32,
    
    
    agentwidth: f32,
    
    shapesetmanager: ShapeSetManager,
    
    
}


impl Pathfinding{
    
    fn new() -> Pathfinding{
        Pathfinding{
            
            totalshapes: 0,
            totallines: 0,
            totalnodes:0,
            
            shapemap: HashMap::new(),
            segmentmap: HashMap::new(),
            nodemap: HashMap::new(),
            
            
            
            shapedbvt: DBVT::new(),
            
            connectiondbvt: DBVT::new(),
            
            realnodedbvt: DBVT::new(),
            
            
            linelengthpernode: 1200.0,
            
            
            agentwidth: 55.0,
            
            
            shapesetmanager: ShapeSetManager::new(),
            
        }
    }
    
    
    
    
    //create two nodes on the edge of a shape, one outside, one inside
    fn createbordernodes(&mut self, nodebase: Point2<f32>, nodenormal: Vector2<f32>, shapeid: u32){        
        
        let insidenodeid = self.totalnodes;        
        self.totalnodes += 1;
        
        let outsidenodeid = self.totalnodes;        
        self.totalnodes += 1;
        
        
        let mut thepathfindinginsidenode = PathfindingNode::newonshape(nodebase, nodenormal, shapeid);
        let mut thepathfindingoutsidenode = PathfindingNode::newonshape(nodebase, nodenormal, shapeid);
        
        
        //make a real node for the two nodes at the agentwidth distance
        let realpoint = thepathfindinginsidenode.createreal(self.agentwidth / 1.9 );
        let realpoint = thepathfindingoutsidenode.createreal(self.agentwidth / 1.9 );
        
        //add it to this objects map of the nodes
        self.nodemap.insert( insidenodeid, thepathfindinginsidenode );
        self.nodemap.insert( outsidenodeid, thepathfindingoutsidenode );
        
        
        
        //get the shape its on
        let mut shapenodesareon = self.shapemap.get_mut(&shapeid).unwrap();
        
        //put the inside and outside node for the shape
        shapenodesareon.addinsideoutsidenodes(&insidenodeid, &outsidenodeid);
        
        
        
        
        
        let ballradius = self.agentwidth / 2.0;
        
        //ball shape for the point
        let nodeshape = Ball::new(ballradius);
        
        //create an iso at the point this node is
        let nodeiso = Isometry2::translation(realpoint.x, realpoint.y);
        
        //create a bounding volume for it
        let thebounding = bounding_volume::aabb(&nodeshape, &nodeiso);
        
        
        //create a dbvt leaf for node 1 at this point
        let theleaf = DBVTLeaf::new( thebounding.clone() , insidenodeid);
        //add this node to the node dbvt
        self.realnodedbvt.insert(theleaf);
        
        //create a dbvt leaf for node 2 at this point
        let theleaf = DBVTLeaf::new( thebounding, outsidenodeid);
        //add this node to the node dbvt
        self.realnodedbvt.insert(theleaf);
        
        
        //get the shapes that both of these nodes intersects with 
        let mut nodeshapeset = self.getshapeintersection(&nodeiso, &nodeshape);
        
        nodeshapeset.insert(shapeid);
        self.shapesetmanager.addnode(insidenodeid, nodeshapeset.clone());
        
        nodeshapeset.remove(&shapeid);
        self.shapesetmanager.addnode(outsidenodeid, nodeshapeset);
        
        
        
        self.setconnections(insidenodeid);
        self.setconnections(outsidenodeid);
        
        
    }
    
    
    /*
    //create a node which isnt on the border of a shape, figure out what shape its on, and create it for there
    fn createnode(&mut self, nodebase: Point2<f32>, nodenormal: Vector2<f32> ) {
        
        let nodeid = self.totalnodes;
        self.totalnodes += 1;
        
        
        let mut thepathfindingnode = PathfindingNode::new(nodebase, nodenormal);
        
        //make a real node for this node at the agentwidth distance
        let realpoint = thepathfindingnode.createreal(self.agentwidth / 1.9 );
        
        self.nodemap.insert( nodeid, thepathfindingnode );
        
        
        
        let ballradius = self.agentwidth / 2.0;
        
        //ball shape for the point
        let nodeshape = Ball::new(ballradius);
        
        //create an iso at the point this node is
        let nodeiso = Isometry2::translation(realpoint.x, realpoint.y);
        
        //create a bounding volume for it
        let thebounding = bounding_volume::aabb(&nodeshape, &nodeiso);
        
        //create a dbvt leaf for this node at this point
        let theleaf = DBVTLeaf::new( thebounding, nodeid);
        
        //add this node to the node dbvt
        self.realnodedbvt.insert(theleaf);
        
        
        //get the shapes that this node intersects with 
        let mut nodeshapeset = self.getshapeintersection(&nodeiso, &nodeshape);
        
        
        self.shapesetmanager.addnode(nodeid, nodeshapeset);
        
        
        
        self.setconnections(nodeid);
        
        
    }
    */
    
    
    
    
    fn addshape(&mut self, shape: ConvexPolygon<f32>, pos: Isometry2<f32>, pathmap: HashMap<u32, f32>, entermap: HashMap<u32, f32> , exitmap: HashMap<u32, f32>) -> u32{
        
        //the id of this shape
        let shapeid = self.totalshapes;
        self.totalshapes += 1;
        
        
        //adding this shape to the dbvt   
        {
            //create a bounding volume for this shape
            let thebounding = bounding_volume::aabb(&shape, &pos);
            //create a dbvt leaf for this node at this point, with id of the shapeid
            let theleaf = DBVTLeaf::new( thebounding, shapeid );
            
            //add this node to the node dbvt
            self.shapedbvt.insert(theleaf);
        }
        
        
        //add the shape to the shape list
        let theshape = PathfindingShape::new(pos, shape.clone(), pathmap, entermap, exitmap);
        self.shapemap.insert( shapeid, theshape);
        
        
        
        //get all the connections this shape intersects with
        //DESTROY them
        {
            
            //create a visitor
            let mut collectedall: Vec<(u32)> = Vec::new();
            
            //i dont need to grow the shape, because when the segment was added to the dbvt
            //it was added as its bounding box grown by the size of agentwidth/2
            let thebounding = shape.aabb(&pos);
            
            let mut thevisitor = BoundingVolumeInterferencesCollector::new( &thebounding, &mut collectedall );
            
            self.connectiondbvt.visit( &mut thevisitor );
            
            
            //check if it actually intersects
            //if it does remove it from the dbvt and from the list of segments
            for possibleintersectid in collectedall{
                use ncollide2d::query::Proximity;
                
                
                let pathsegment = self.segmentmap.get(&possibleintersectid).expect("why is this dbvt val not in the map of segments");
                
                let segment = &pathsegment.segment;
                
                //see if they actually come within proximity of each other                
                let theproximity = proximity(
                    &Isometry2::identity(),
                    segment, 
                    &pos, 
                    &shape, 
                    self.agentwidth / 2.0
                );
                
                
                //if it is intersecting
                if (theproximity == Proximity::Intersecting ){
                    
                    self.destroysegment( &possibleintersectid );
                    
                }
                
                
                
                
            }
            
            
            
            
        }
        
        
        
        
        //get all the nodes this shape intersects with
        //and add to all their shapesets, this shape 
        {
            
            //create a visitor
            let mut collectedall: Vec<(u32)> = Vec::new();
            
            //i dont need to grow the shape by agentwidth, because i already did that when
            //adding the nodes to the dbvt, their bounding is the boudning of their full sized shape
            let thebounding = shape.aabb(&pos);
            
            let mut thevisitor = BoundingVolumeInterferencesCollector::new( &thebounding, &mut collectedall);
            
            self.realnodedbvt.visit( &mut thevisitor );
            
            
            
            //check if it actually intersects
            //if it does add this shape to its shape set
            //then recalculate its shape set connections
            for possibleintersectnodeid in collectedall{
                
                let pathnode = self.nodemap.get(&possibleintersectnodeid).expect("why is this dbvt val not in the map of nodes");
                
                
                let ballradius = self.agentwidth / 2.0;
                //ball shape for the point
                let nodeshape = Ball::new(ballradius);
                let nodeiso = Isometry2::translation(pathnode.realpoint.x, pathnode.realpoint.y);
                
                
                //see if they actually intersect
                let isincontact = query::contact(&pos, &shape, &nodeiso, &nodeshape, 0.0);
                
                if (isincontact.is_some()){
                    
                    self.shapesetmanager.addtoshapeset(possibleintersectnodeid, shapeid);
                    
                    self.setconnections(possibleintersectnodeid);
                    
                }
                
                
            }
            
            
            
            
            
        }
        
        
        
        
        
        
        
        //get the positions for each of the nodes that should be made
        //and create them
        {
            
            //the list of positions of nodes bases, and their normals
            let mut nodepostocreate: Vec< (Point2<f32>, Vector2<f32>) > = Vec::new();
            
            //fill the list
            {
                //the list of points of this shape
                let thepoints = shape.points();
                let isometry = pos;
                
                for iternumb in 0..thepoints.len(){
                    
                    let point1id = (iternumb) % thepoints.len();
                    let point2id = (iternumb +1) % thepoints.len();
                    let point3id = (iternumb +2) % thepoints.len();
                    
                    let point1 = isometry.transform_point(&thepoints[point1id]);
                    let point2 = isometry.transform_point(&thepoints[point2id]);
                    let point3 = isometry.transform_point(&thepoints[point3id]);
                    
                    
                    let cursegment = Segment::new(point1, point2);
                    
                    let curnormal: Unit<Vector2<f32>> = cursegment.normal().expect("this segment not have a normal");
                    
                    let curscaleddirection = cursegment.scaled_direction();
                    
                    //get the length of this segment
                    let length = cursegment.length();
                    
                    //the number of nodes that need to be created for this segment
                    let numberofnodes = (length / self.linelengthpernode) as u32 + 2;
                    
                    
                    for curstep in 0..numberofnodes{
                        
                        
                        //make the node at the position of point 1
                        let mut nodeposition = point1;
                        
                        //add to it a portion down the line depending on the iteration
                        nodeposition += (curscaleddirection * ( (curstep as f32) / ((numberofnodes -1) as f32) ) );
                        
                        //DONT add to it, the agent width * the direction of this segments normal
                        //nodeposition += curnormal.into_inner() * self.agentwidth * 1.05;
                        
                        
                        nodepostocreate.push( (nodeposition, curnormal.into_inner()) );
                        
                    }
                    
                    
                    
                    
                    let nextsegment = Segment::new(point2, point3);
                    let nextnormal = nextsegment.normal().expect("this segment not have a normal");
                    
                    
                    //the number of connecting nodes that will be made to connect this segment and the next one
                    //at point2 , between point 1 and 3
                    let connectingslerps = 1;
                    
                    for curstep in 0..connectingslerps{
                        
                        let slerpnormal = curnormal.slerp(&nextnormal,     ((curstep + 1) as f32) / ((connectingslerps + 1) as f32) );
                        
                        
                        let nodeposition = point2;// + ( slerpnormal.into_inner() * self.agentwidth/2.0 ) * 1.05;
                        
                        
                        nodepostocreate.push( (nodeposition, slerpnormal.into_inner()) );
                        
                    }
                    
                }
            }
            
            
            //now create all of the nodes at those positions and add them to the collision world
            for (nodepos, nodenormal) in nodepostocreate{
                
                
                self.createbordernodes( nodepos , nodenormal , shapeid);
                
                
            }
            
        }
        
        
        
        
        
        shapeid
    }
    
    
    
    //return the lists of shapepath ids that this shape intersects with (works with nodes)
    fn getshapeintersection(&self, isometry: &Isometry2<f32>, shape: &Shape<f32>) -> BTreeSet<u32>{
        
        //create a visitor
        let mut collectedall: Vec<(u32)> = Vec::new();
        
        let thebounding = shape.aabb(&isometry);
        
        let mut thevisitor = BoundingVolumeInterferencesCollector::new( &thebounding, &mut collectedall);
        
        
        self.shapedbvt.visit( &mut thevisitor);
        
        
        let mut toreturn = BTreeSet::new();
        
        
        //check if it actually intersects
        //if it does put it in the "toreturn" set
        for possibleintersectid in collectedall{
            
            let pathshape = self.shapemap.get(&possibleintersectid).expect("why is this dbvt val not in the map of shapes");
            
            let othershape = &pathshape.shape;
            let otheriso = &pathshape.pos;
            
            //see if they actually intersect
            let isincontact = query::contact(&isometry, shape, &otheriso, othershape, 0.0);
            
            if (isincontact.is_some()){
                
                //if they do, add that to the intersected shapes to return
                toreturn.insert(possibleintersectid);
                
            }
            
        }
        
        toreturn
        
    }
    
    




    
    //destroy every connection thats attached to this node
    fn destroynodeconnections(&mut self, nodeid: &u32){
        
        let mut thenode = self.nodemap.get_mut(nodeid).unwrap();

        //get every segment attached to it and destroy it
        let mut attachedsegments = thenode.get_segment_ids();

        for cursegmentid in attachedsegments{
            self.destroysegment( &cursegmentid );

        }

        
    }

    //destroy a single segment
    fn destroysegment(&mut self, segmentid: &u32){

        let mut thesegment = self.segmentmap.get_mut(segmentid).unwrap();

        let dbvtleafid = thesegment.leafid;

        //get the nodes on this segment, remove this segment from their connected segments
        let node1id = thesegment.node1id;
        let node2id = thesegment.node2id;

        let mut node1 = self.nodemap.get_mut(&node1id).unwrap();
        node1.removeattachedsegment(segmentid);
        
        let mut node2 = self.nodemap.get_mut(&node2id).unwrap();
        node2.removeattachedsegment(segmentid);



        //remove the reference to this segment from the dbvt list
        self.connectiondbvt.remove(dbvtleafid);

        
        self.segmentmap.remove(segmentid);

    }
    
    
    
    
    //destroy every segment connected to this node
    //then recreate every segment for this node
    fn setconnections(&mut self, nodeid: u32){
        
        
        self.destroynodeconnections(&nodeid);
        
        
        //get all other nodes in the same shape set as this node
        let thenodes = self.shapesetmanager.getnodesinsameshapeset(&nodeid);
        
        let thisnode = self.nodemap.get(&nodeid).unwrap();
        let thisnodepos = thisnode.realpoint;
        
        //create a segment between every node in the same shape set
        for othernodeid in thenodes{
            
            //get the other node
            let othernode = self.nodemap.get(&othernodeid).unwrap();
            let othernodepos = othernode.realpoint;
            
            
            let shapesegment = Segment::new(thisnodepos, othernodepos);
            
            let mut collectedall: Vec<(u32)> = Vec::new();
            
            //create a bounding and grow it by agentwidth / 2
            let mut thebounding : AABB<f32> = shapesegment.local_bounding_volume();
            thebounding.loosen(self.agentwidth / 2.0 );
            
            let mut thevisitor = BoundingVolumeInterferencesCollector::new( &thebounding, &mut collectedall);
            
            self.shapedbvt.visit( &mut thevisitor);
            
            
            //if this segment intersects with any shapes, dont create it
            let mut validconnection = true;
            for shapeid in collectedall{
                
                
                use ncollide2d::query::Proximity;
                
                //see if the shape and the segment are actually within proximity
                let thepathshape = self.shapemap.get(&shapeid).unwrap();
                
                let theproximity = proximity(
                    &Isometry2::identity(),
                    &shapesegment, 
                    &thepathshape.pos, 
                    &thepathshape.shape, 
                    self.agentwidth / 2.0
                );
                
                //if the shape this segment intersects with is a shape not in the shapeset both are already in
                if ( ! self.shapesetmanager.isinsideshape( shapeid , nodeid ) ){
                    
                    //if it is intersecting
                    if (theproximity == Proximity::Intersecting ){
                        
                        //then this segment should not be made
                        validconnection = false;
                        
                    }
                    
                    
                }
                
                
                
            }
            
            if (validconnection){
                
                
                let segmentid = self.totallines;
                self.totallines += 1;
                
                
                let mut connectionevalmap: HashMap<u32, f32> = HashMap::new();
                
                //get the shapeset that both of the nodes are in (because theyre in the same shapeset)
                for curshapeid in self.shapesetmanager.getshapesetofnode(&nodeid){
                    
                    //get the shape
                    let curshape = self.shapemap.get(&curshapeid).unwrap();
                    
                    //add the shapesevalmap to the evalmap for this connection
                    addevalmaps(&mut connectionevalmap, &curshape.pathmap);
                    
                }
                
                
                //create a bounding volume for the segment
                let mut thebounding: AABB<f32> = shapesegment.local_bounding_volume();
                thebounding.loosen(self.agentwidth / 2.0 );
                
                //create a dbvt leaf for this node at this point
                let theleaf = DBVTLeaf::new( thebounding, segmentid);
                let dbvtleafid = self.connectiondbvt.insert(theleaf);
                
                
                //create a segment
                let pathsegment = PathfindingSegment::new(
                    shapesegment,
                    dbvtleafid,
                    nodeid,
                    othernodeid,
                    connectionevalmap
                );
                
                self.segmentmap.insert(segmentid, pathsegment);
                
                
                let mut thisnode = self.nodemap.get_mut(&nodeid).unwrap();
                thisnode.addsegment(segmentid);
                
                let mut othernode = self.nodemap.get_mut(&othernodeid).unwrap();
                othernode.addsegment(segmentid);
                
            }
            
            
        }
        
        
        
        
        //set the connection for the outside to this inside node or inside to this outside node
        //get if this shape is on a node and the shape id of the shape its on
        if let Some(shapeid) = self.nodemap.get(&nodeid).unwrap().optionshapeon{
            
            let thisnode = self.nodemap.get(&nodeid).unwrap();
            
            let segmentid = self.totallines;
            self.totallines += 1;
            
            
            //get the shape
            let theshape = self.shapemap.get(&shapeid).unwrap();
            
            
            
            //get the other node
            //get the evalmap of going from node 1 to 2 and node 2 to 1
            
            let (othernodeid, nodetoothercostmap, othertonodecostmap) = theshape.getnodeandcostmap(&nodeid);
            
            //what data is assocaited with the creation of a segment?
            
            //self.segmentmap
            //self.nodemap all the nodes have a reference to the segmentid between them and another node
            //self.connectiondbvt for a shape for the segment
            
            
            let thisnodepos = thisnode.realpoint;
            
            let othernode = self.nodemap.get(&othernodeid).unwrap();
            let othernodepos = othernode.realpoint;
            
            
            let shapesegment = Segment::new(thisnodepos, othernodepos);
            
            
            //create a bounding volume for the segment
            let mut thebounding: AABB<f32> = shapesegment.local_bounding_volume();
            thebounding.loosen(self.agentwidth / 2.0 );
            
            //create a dbvt leaf for this node at this point
            let theleaf = DBVTLeaf::new( thebounding, segmentid);
            let dbvtleafid = self.connectiondbvt.insert(theleaf);
            
            
            //create a segment
            let pathsegment = PathfindingSegment::newdirectional(
                shapesegment,
                dbvtleafid,
                nodeid,
                othernodeid,
                nodetoothercostmap,
                othertonodecostmap
            );
            
            self.segmentmap.insert(nodeid, pathsegment);
            
            
            //RIGHT NOw the issue is that for some reason, segments are being attached to nodes which
            //are not connected to them, find out why
            //or are being given the wrong node ids for the nodes theyre attached to
            
            
            
            let mut thisnode = self.nodemap.get_mut(&nodeid).unwrap();
            thisnode.addsegment(segmentid);
            
            let mut othernode = self.nodemap.get_mut(&othernodeid).unwrap();
            othernode.addsegment(segmentid);
            
            
        }
        
        
        
        
    }
    
    
    
    
    
    
    



    //draws all of the shapes and all of the nodes and all of the connections
    //done
    fn drawstate(&self){
        
        let mut imageoutput: Vec<(Vec<(u8,u8,u8)>)>  = Vec::new();
        
        let leftedge = -100;
        let topedge = -100;
        let bottomedge = 1500;
        let rightedge = 1500;
        
        let xsize = rightedge - leftedge;
        let ysize = bottomedge - topedge;
        
        
        //fill the list and set the background colour
        for y in 0..ysize{
            
            let mut thisrow: Vec<(u8,u8,u8)> = Vec::new();
            for x in 0..xsize{
                thisrow.push((70,10,10));
            }
            imageoutput.push(thisrow);
            
        }
        
        
        //iterate through the shapes
        
        for (_, curpathshape) in self.shapemap.iter(){
            
            
            let shapeshape = &curpathshape.shape;
            let shapepos = &curpathshape.pos;
            
            
            
            drawconvex(&mut imageoutput, shapeshape, shapepos, leftedge, topedge, (200,200,200) );
            
        }
        
        
        //println!("IM HERE ICHI");
        
        //iterate through the connections
        
        println!("this is the size is{:?}", self.segmentmap.len());
        
        for (_, cursegment) in self.segmentmap.iter(){
            
            
            
            //point
            let point1 = cursegment.segment.a();
            
            let point2 = cursegment.segment.b();
            
            let point1x = point1.x as i16;
            let point1y = point1.y as i16;
            
            let point2x = point2.x as i16;
            let point2y = point2.y as i16;
            
            drawline(&mut imageoutput, (point1x, point1y),   ( point2x, point2y )  , leftedge, topedge, (100,100,100) );
            
            
            
        }
        
        
        
        //turn the image output into a flat u8 array
        let mut buffer: Vec<u8> = Vec::new(); // Generate the image data
        
        for currow in imageoutput{
            
            for currgb in currow{
                
                buffer.push(currgb.0);
                buffer.push(currgb.1);
                buffer.push(currgb.2);
                
            }
            
        }
        
        
        
        // Save the buffer as "image.png"
        image::save_buffer("statedrawn.png", &buffer, xsize as u32, ysize as u32, image::ColorType::Rgb8).unwrap();
        
        
    }
    
    //get the path
    fn getpath(&self, nodeid: u32, evalmap: HashMap< u32, f32>) -> Vec< Point2<f32> >{
        
        
        
        //the list of paths
        let mut listofpaths: ListOfPaths = ListOfPaths::new();
        
        listofpaths.addfirstpath(nodeid);
        
        
        //how do I know when im done?
        //cuz this just runs on forever
        
        //gets the lowest value valid path
        //
        let mut thelowestpath = 0;
        
        
        
        
        for x in 0..100000{
            
            //store the cost of the last node, so i know at the end, what
            //the minimum value path is
            let lastcost = 0.0;
            
            
            //println!("This is it{:?}", listofpaths.pathtonode);
            
            //println!("This is it originator{:?}", listofpaths.pathtooriginator);
            
            //println!("This is it to path{:?}", listofpaths.pathtopaths);
            
            if let Some( (pathcost, pathid) ) = listofpaths.getlowestvalidpath(){
                
                
                //println!("cost, then id, {:?}, {:?}", pathcost, pathid);
                
                thelowestpath = pathid;
                
                
                
                
                //get the node that this path ends on
                
                let currentnodeid = listofpaths.getnodeofpath( &pathid );
                
                //println!("This is the current node {:?}", currentnodeid);
                
                
                
                //get all the segments attached to the current node
                
                //get the list of segments leaving this node
                let currentnode = self.nodemap.get(&currentnodeid).unwrap();
                
                //get the segmentids attached to this node
                let setofsegmentsattached = currentnode.get_segment_ids();
                
                
                for cursegmentid in setofsegmentsattached{
                    
                    //println!("the segment attached to this node{:?}", cursegmentid);
                    
                    //get the segment
                    let cursegment = self.segmentmap.get(&cursegmentid).unwrap();
                    
                    //println!("cursegmentid:{:?}", cursegmentid);
                    //println!("curnodeid:{:?}", currentnodeid);
                    //println!("cursegment:{:?}", cursegment);
                    
                    
                    //for this segment, get the node on the other end, along with the value of it
                    let (newnodeid, curpathvalue) = cursegment.getothernodeandvalue(&currentnodeid, &evalmap);
                    
                    //println!("the node the segment is attached to and value{:?}, + {:?}", curpathvalue, newnodeid);
                    
                    let newpathvalue = curpathvalue + pathcost;
                    
                    //add this to the list of paths
                    println!("I got here, the main loop is breaking after 1 iteration, so what is the problem?");
                    listofpaths.addpath(newpathvalue, pathid, newnodeid);
                    
                }
                
            }
            else
            {    
                println!("no lowest cost paths left");
                break;
            }
        }
        
        //print the list of paths taken from the origin node to get to this one
        listofpaths.printfullpath(&thelowestpath);
        
        //listofpaths.removepathandsuccessors(&0);
        
        
        println!("This is it to node{:?}", listofpaths.pathtonode);
        
        println!("");
        println!("This is it originator{:?}", listofpaths.pathtooriginator);
        println!("");
        println!("This is it to path{:?}", listofpaths.pathtopaths);
        println!("");
        println!("This is the paths to its cost{:?}", listofpaths.pathtocost);
        println!("");
        
        println!("");
        println!("");
        
        
        
        listofpaths.printfullpath(&thelowestpath);
        
        self.drawfullpath(  &listofpaths, &thelowestpath );
        
        
        /*
        loop{
            
            //get the current lowest cost path
            let (curpathvalue , curpathid) = listofpaths.getlowestvalidpath();
            
            //get the nodeid that the current path ends on
            let curnodeid = pathtonode.get(&curpathid).unwrap();
            //get the node that the current path ends on
            let curnode = self.nodemap.get(&curnodeid).unwrap();
            //get all the segments that are connected to this node
            let connectedsegmentids = curnode.get_segment_ids();
            
            //for all the segments connected to this node
            for cursegmentid in connectedsegmentids{
                
                let cursegment = self.segmentmap.get(&cursegmentid).unwrap();
                
                let nextnodeid = cursegment.getothernode(curnodeid);
                
                let oldpathvalue = curpathvalue;
                
                //get the value of the new path
                let newpathvalue = oldpathvalue + cursegment.getvalue(&evalmap);
                
                
                
                //add this path
                listofpaths.addpath(newpathvalue, curpathid, nextnodeid);
                
                
                
            }
            
            break;
        }
        */
        
        
        
        
        Vec::new()
        
        
    }
    
    //draw the full path with the listofpaths, and the pathid
    fn drawfullpath(&self, listofpaths: &ListOfPaths , pathid: &u32){
        
        
        //get the set of connecions
        let setofconnections = listofpaths.getsetofconnections(&pathid);
        
        println!("This is the set of connections: {:?}", setofconnections);
        
        
        let mut imageoutput: Vec<(Vec<(u8,u8,u8)>)>  = Vec::new();
        
        let leftedge = -100;
        let topedge = -100;
        let bottomedge = 1500;
        let rightedge = 1500;
        
        let xsize = rightedge - leftedge;
        let ysize = bottomedge - topedge;
        
        
        //fill the list and set the background colour
        for y in 0..ysize{
            
            let mut thisrow: Vec<(u8,u8,u8)> = Vec::new();
            for x in 0..xsize{
                thisrow.push((70,10,10));
            }
            imageoutput.push(thisrow);
            
        }
        
        
        
        
        //iterate through the connections
        println!("this is the size is{:?}", self.segmentmap.len());
        
        for (_, cursegment) in self.segmentmap.iter(){
            
            let mut thecolour = (100,100,100);
            
            
            let point1 = cursegment.segment.a();
            
            let point2 = cursegment.segment.b();
            
            let point1x = point1.x as i16;
            let point1y = point1.y as i16;
            
            let point2x = point2.x as i16;
            let point2y = point2.y as i16;
            
            drawline(&mut imageoutput, (point1x, point1y),   ( point2x, point2y )  , leftedge, topedge, thecolour );
            
            
            
        }
        
        
        
        
        //iterate through the shapes
        for (_, curpathshape) in self.shapemap.iter(){
            
            
            let shapeshape = &curpathshape.shape;
            let shapepos = &curpathshape.pos;
            
            
            
            drawconvex(&mut imageoutput, shapeshape, shapepos, leftedge, topedge, (200,200,200) );
            
        }
        
        
        
        
        
        //iterate through the connections again for the coloured ones
        for (_, cursegment) in self.segmentmap.iter(){
            
            let mut thecolour = (200,200,100);
            
            if (setofconnections.contains( &(cursegment.node1id,cursegment.node2id) )){
                
                let point1 = cursegment.segment.a();
                
                let point2 = cursegment.segment.b();
                
                let point1x = point1.x as i16;
                let point1y = point1.y as i16;
                
                let point2x = point2.x as i16;
                let point2y = point2.y as i16;
                
                drawline(&mut imageoutput, (point1x, point1y),   ( point2x, point2y )  , leftedge, topedge, thecolour );
                
            }
            
            if (setofconnections.contains( &(cursegment.node2id,cursegment.node1id) )){
                
                
                let point1 = cursegment.segment.a();
                
                let point2 = cursegment.segment.b();
                
                let point1x = point1.x as i16;
                let point1y = point1.y as i16;
                
                let point2x = point2.x as i16;
                let point2y = point2.y as i16;
                
                drawline(&mut imageoutput, (point1x, point1y),   ( point2x, point2y )  , leftedge, topedge, thecolour );
                
            }
            
            
            
        }
        
        
        
        
        
        //turn the image output into a flat u8 array
        let mut buffer: Vec<u8> = Vec::new(); // Generate the image data
        
        for currow in imageoutput{
            
            for currgb in currow{
                
                buffer.push(currgb.0);
                buffer.push(currgb.1);
                buffer.push(currgb.2);
                
            }
            
        }
        
        
        
        // Save the buffer as "image.png"
        image::save_buffer("statedrawnpath.png", &buffer, xsize as u32, ysize as u32, image::ColorType::Rgb8).unwrap();
        
        
        
    }
    
    //get the id of every path that was taken to get to this path (not including itself)
    fn getsetofpathstaken(startpathid: &u32, pathtooriginator: &HashMap<u32, u32>) -> HashSet<u32>{
        
        let mut returnset = HashSet::new();
        
        let mut currentpathid = startpathid;
        
        //follow back until it gets to a pathvalue that does not exist, that means that pathvalue doesnt have an originator
        
        loop{
            
            
            
            if let Some(pathbeforeid) = pathtooriginator.get(currentpathid){
                
                currentpathid = pathbeforeid;
                
                returnset.insert(*currentpathid);
                
            }
            else{
                
                break;
            }
            
            
        }
        
        
        
        returnset
        
    }
    
}



struct ListOfPaths{
    
    currentpathid: u32,
    
    //for each node, the list of pathids that end on this node
    nodetopath: HashMap< u32, HashSet<u32> >,
    //for each path id, map the path id to the node that this path ends on
    pathtonode: HashMap< u32 , u32 >,
    
    //for every path with a sucessor, the list of sucessors
    pathtopaths: HashMap< u32, HashSet<u32> >,
    
    //a map from each path, to the path it grew out of
    pathtooriginator: HashMap< u32, u32>, 
    
    //cost to get to this path, path id
    listofpathcosts: HeapWithValue,
    
    //the map of the costs of each path
    pathtocost: HashMap<u32, f32>,
    
}

impl ListOfPaths{
    //IMPORTANT INVARIANT
    /*
    AT ANY TIME, THERE CAN ONLY BE ONE CHAIN OF PATHS THAT ARE ON A NODE
    A NODE MAY ONLY HAVE ONE PATH THAT ENDS ON IT
    OR MULTIPLE PATHS THAT END ON IT, ONLY WHEN THOSE PATHS ARE A PART OF THE SAME CHAIN
    (THERE CAN ONLY BE ONE PATH ENDING ON A NODE, WHO DOES NOT HAVE PROGENITORS THAT END ON THE SAME NODE)
    */
    fn new() -> ListOfPaths{
        
        ListOfPaths{
            currentpathid:0,
            nodetopath: HashMap::new(),
            pathtonode: HashMap::new(),
            pathtooriginator: HashMap::new(),
            listofpathcosts: HeapWithValue::new(),
            pathtopaths: HashMap::new(),
            pathtocost: HashMap::new(),
        }
        
    }
    
    fn printfullpath(&self, pathid: &u32){
        
        println!("this pathid: {:?} , and its nodeid: {:?}", pathid, self.pathtonode.get(&pathid).unwrap());
        
        println!("and its pathvalue : {:?}", self.pathtocost.get(&pathid).unwrap());
        
        
        if let Some( newpathid ) = self.pathtooriginator.get(pathid){
            
            
            
            self.printfullpath(newpathid);
            
        }
        else
        {
            
            return( () )
        } 
        
        
        
        
    }
    
    //return all the set of connections from this node back to its predecessor
    fn getsetofconnections(&self, pathid: &u32) -> HashSet<(u32, u32)> {
        
        
        //if the current path has an originator
        if let Some( originatorpathid ) = self.pathtooriginator.get(pathid){
            
            //get the hashset from all nodes up until the one before this one
            let mut toreturn =  self.getsetofconnections(originatorpathid);
            
            //get the ids of the nodes theyre on and add it to the hashset
            let originatornodeidonpathid = self.pathtonode.get(originatorpathid).unwrap();
            let nodeidonpathid = self.pathtonode.get(pathid).unwrap();
            
            toreturn.insert( (*originatornodeidonpathid, *nodeidonpathid) );
            
            //return it
            return(toreturn);
        }
        else
        {
            //weve got to the bottom, so return an empty hashset
            return(HashSet::new());
            
            
        }
        
        
    }
    
    //get the node this path ends on
    fn getnodeofpath(&self, pathid: &u32) -> u32{
        
        if let Some(nodeid) = self.pathtonode.get(pathid){
            
            *nodeid
        }
        else{
            
            panic!("path not found in the 'pathtonode' list");
        }
    }
    
    fn getlowestvalidpath(&mut self) -> Option<(f32, u32)>{
        
        self.listofpathcosts.pop()
        
        //A PATH CAN ONLY BE "GOTTEN" ONCE
        //SO ONCE A PATH IS GIVEN WITH THIS FUNCTION, SET THAT THIS PATH CANT BE RETURNED BY THSI FUNCTION AGAIN
        //THIS RULE IS ALWAYS TRUE
        //YOU MAY RETURN MULTIPLE PATHS THAT END ON THE SAME NODE, BUT NEVER THE SAME PATH TWICE
        
    }
    
    
    //check if the X path preceeds or IS the Y path
    fn doesXproceedY(&self, xpathid: &u32, ypathid: &u32 ) -> bool{        
        
        if (xpathid == ypathid){
            return(true)
        }
        
        else{
            
            //get the path preceeding the ypath if it exists
            //or return FALSE if the y path has nothing proceeding it
            
            if let Some( newypathid ) = self.pathtooriginator.get(ypathid){
                
                return( self.doesXproceedY(xpathid, newypathid )  )
                
            }
            else
            {
                return(false)
            }
            
        }        
        
    }
    
    
    //adding the beginning path that has no predecessor
    fn addfirstpath(&mut self, nodeid: u32){
        
        //the pathvalue of the first path is 0
        let pathvalue = 0.0;
        
        //now create this path, and add it to all the lists
        let currentpathid = self.currentpathid;
        self.currentpathid += 1;
        
        if let Some(curpathsonnode) = self.nodetopath.get_mut(&nodeid){
            curpathsonnode.insert(currentpathid);
        }
        else{
            let mut thehashset = HashSet::new();
            thehashset.insert(currentpathid);
            self.nodetopath.insert(nodeid, thehashset);
        }
        
        //this path doesnt have an originator
        //self.pathtooriginator.insert(previouspathid ,currentpathid);
        //self.pathtopaths.get_mut(&previouspathid).unwrap().insert(currentpathid);
        
        self.nodetopath.get_mut(&nodeid).unwrap().insert(currentpathid);
        self.pathtonode.insert(currentpathid, nodeid);
        self.listofpathcosts.insert(pathvalue, currentpathid);
        self.pathtocost.insert(currentpathid, pathvalue);
        
        //create a path to paths for this node
        self.pathtopaths.insert(currentpathid, HashSet::new());
        
    }
    
    //for a new path to be added
    //with its pathvalue, its progenitor path, and the node its ending on
    //this is the ONLY means and function to add a path to the list of paths
    fn addpath(&mut self, pathvalue:f32, previouspathid: u32, nodeid: u32){
        
        //if this is the lowest cost node on this point
        //and if the only nodes on this point are predecessors of this path
        //then this path can be made
        let mut canbemade = false;
        
        //get all of the paths that end on this node
        let pathsonsamenodeoption = self.nodetopath.get(&nodeid);
        
        //get if this node can be made
        if let Some(pathsonsamenode)  = pathsonsamenodeoption{
            
            let mut lowestpathcostatnode = 100000000000.0;
            
            let pathidendingonsamenodeset = self.nodetopath.get(&nodeid).unwrap();
            
            //first get the path cost of the LOWEST cost path on this node
            for pathidendingonsamenode in pathidendingonsamenodeset{
                
                
                if (self.pathtocost.get(pathidendingonsamenode).unwrap() < &lowestpathcostatnode){
                    
                    lowestpathcostatnode = *self.pathtocost.get(pathidendingonsamenode).unwrap();
                    
                }
                
                //println!("the value is {:?}", lowestpathcostatnode);
                
            }
            //if this path is a lower cost than any other path ending on the node it ends on
            if ( pathvalue < lowestpathcostatnode ){
                canbemade = true;
            }
        }
        //if it hasnt been created yet
        else{
            //then there are no nodes for this 
            canbemade = true;
            
        }
        
        
        let mut pathvalue = pathvalue;
        //check if for this path to be created, if this path has already visited the node that it is trying to visit now
        if ( self.hasalreadyvisitednode(&previouspathid, &nodeid) ){
            
            //make the pathvalue equal to 0.01
            pathvalue = 0.01;
            
        }
        
        
        //if this path can be made
        if (canbemade){
            
            
            //if this node hasnt been created in the nodetopath map
            if ( ! self.nodetopath.contains_key(&nodeid) ){
                self.nodetopath.insert(nodeid, HashSet::new());
            }
            
            let pathidendingonsamenodeset = self.nodetopath.get(&nodeid).unwrap().clone();
            
            
            //if there are paths ending on this node
            //if any of these paths are NOT a part of this nodes predecessors, remove them before creating creating this new path
            
            
            //for each of the paths that end on this node
            for pathidendingonsamenode in pathidendingonsamenodeset{
                
                //println!("pathendingonsamenode{:?} and the cur is {:?}", pathidendingonsamenode, previouspathid);
                
                if (self.doesXproceedY( &pathidendingonsamenode, &previouspathid)){
                    
                    //println!("do i proceed?");
                    
                    //if this node is a predecessor to or the previous path
                    //do nothing, you dont need to, and shouldnt remove this path before creating its successors
                    
                }
                else{
                    
                    //if this node is NOT a predecessor to this new path
                    //REMOVE this node
                    self.removepathandsuccessors(&pathidendingonsamenode);
                    
                }
            }
            
            
            
            //now create this path, and add it to all the lists
            
            let currentpathid = self.currentpathid;
            self.currentpathid += 1;
            
            if let Some(curpathsonnode) = self.nodetopath.get_mut(&nodeid){
                curpathsonnode.insert(currentpathid);
            }
            else{
                let mut thehashset = HashSet::new();
                thehashset.insert(currentpathid);
                self.nodetopath.insert(nodeid, thehashset);
            }
            
            self.pathtooriginator.insert(currentpathid ,previouspathid);
            self.pathtopaths.get_mut(&previouspathid).unwrap().insert(currentpathid);
            self.nodetopath.get_mut(&nodeid).unwrap().insert(currentpathid);
            self.pathtonode.insert(currentpathid, nodeid);
            self.listofpathcosts.insert(pathvalue, currentpathid);
            self.pathtocost.insert(currentpathid, pathvalue);
            
            //create a path to paths for this node
            self.pathtopaths.insert(currentpathid, HashSet::new());
            
            
        }
        
        
    }
    
    
    //if this path or its predecessors has already visited this node
    fn hasalreadyvisitednode(&self, pathid:&u32, nodeid:&u32) -> bool{
        
        
        //get the node this path is on
        
        let pathsnodeid = self.pathtonode.get(pathid).unwrap();
        
        if (pathsnodeid == nodeid){
            return(true)
        }
        else{
            
            
            //if the current path has an originator
            if let Some( originatorpathid ) = self.pathtooriginator.get(pathid){
                
                return( self.hasalreadyvisitednode(originatorpathid, nodeid )  )
                
            }
            else
            {
                //return that weve got to the bottom, and it hasnt already visited this node
                return(false)
            }
            
        }        
        
    }
    
    
    //remove a path and all the paths that come out of it
    //(do this when the path to this node is being replaced with a path that is better)
    fn removepathandsuccessors(&mut self, pathid: &u32 ){
        
        //for all of the paths stemming from this path
        let pathsstemmingoption = self.pathtopaths.get(pathid).clone();
        
        
        //if there are paths that stem from this one:
        if let Some(pathsstemming) = pathsstemmingoption.clone(){
            
            for pathstemmingid in pathsstemming.clone(){
                
                self.removepathandsuccessors(&pathstemmingid);
            }
            
        }
        
        //remove this path
        
        self.pathtopaths.remove(pathid);
        
        let nodeid = self.pathtonode.remove(pathid).unwrap();
        
        self.nodetopath.get_mut(&nodeid).unwrap().remove(pathid);
        
        self.pathtooriginator.remove(pathid);
        
        //REMOVE THIS NODE FROM THE self.listofpathcosts
        self.listofpathcosts.remove(pathid);
    }
    
    
    
}


//this is a heap that has associated with it
struct HeapWithValue{
    
    //the list of the values and the nodes
    mainlist: BTreeMap< NotNan<f32> , HashSet<u32> >,
    
    idtovalue: HashMap< u32, NotNan<f32> >,
    
}


impl HeapWithValue{
    
    fn new() -> HeapWithValue{
        
        HeapWithValue{ mainlist: BTreeMap::new(), idtovalue: HashMap::new() }
        
    }
    
    //get the pathvalue of the highest cost node, and the id of the associated node
    fn pop(&mut self) -> Option<( f32 , u32 )>{
        
        
        if let Some( ( key, _ ) ) = self.mainlist.first_key_value(){
            
            let key = key.clone();
            let nodeset = self.mainlist.get_mut(&key).unwrap();
            
            
            let pathvalue = key.into_inner();
            
            
            let mut maybereturnnodeid : Option<u32> = None;
            
            if (nodeset.len() == 0){
                
                println!("the list{:?}", self.mainlist);
                panic!("WHAT IS GOING ON, why is there a key to an empty set of paths");
            }
            
            //iterate through just to grab and remove one value from the nodeset
            for curvalue in nodeset.iter(){
                
                maybereturnnodeid = Some(*curvalue);
                
                
                break;
                
            }
            
            if let Some(returnnodeid) = maybereturnnodeid{            
                
                nodeset.remove(&returnnodeid);
                
                if (nodeset.len() == 0){
                    self.mainlist.remove(&key);
                }
                
                
                //remove that removed value from the idtovalue list
                self.idtovalue.remove(&returnnodeid);
                
                
                //return the pathvalue and the id of the node
                return( Some((pathvalue, returnnodeid)) ); 
                
            }
            else{
                
                //return(None);
                panic!("the return node id didnt exist in thsi set of nodes");
            }
        }
        else{
            
            return(None);
        }
        
        
    }
    
    
    //given the id of a path/node remove that value
    fn remove(&mut self, nodeid: &u32){
        
        
        //if this path hasnt already been removed
        if (self.idtovalue.contains_key(nodeid)){
            
            //use std::env;
            //let key = "KEY";
            //env::set_var("RUST_BACKTRACE", "1");
            
            //println!("before removing the path {:?}", self.mainlist);
            
            
            //remove it from the idtovalue list
            let thekey = self.idtovalue.remove(&nodeid).unwrap();
            
            
            //then remove it from the mainlist
            let idset = self.mainlist.get_mut(&thekey).unwrap();
            
            idset.remove(nodeid);
            
            //if that key for the mainlist is empty, remove it
            if (idset.len() == 0){
                
                self.mainlist.remove(&thekey);
            }
            
            //this is the list after removing this path
            //println!("THIS IS THE PATHKEY:{:?}", thekey);
            //println!("THIS IS THE PATHKEY:{:?}", nodeid);
            
            
            
            //println!("after removing the path {:?}", self.mainlist);
            //println!("-");println!("-");println!("-");println!("-");println!("-");
        }
        
    }
    
    
    //add in the nodeid with its associated pathvalue
    fn insert( &mut self, pathvalue: f32 , nodeid: u32 ){
        
        let key = NotNan::new( pathvalue ).expect("nan error");
        
        //check if it doesnt exist, make it first
        if (! self.mainlist.contains_key(&key)){
            
            self.mainlist.insert(key, HashSet::new());
        }
        
        
        self.mainlist.get_mut(&key).unwrap().insert(nodeid);
        
        self.idtovalue.insert(nodeid, key);
        
        
    }
    
    
    
}



//TEST THIS
//given two evalmaps, add the second one to the first one
fn addevalmaps(evalmap1: &mut HashMap<u32, f32>, evalmap2: &HashMap<u32, f32>) {
    
    //for each of the values in evalmap2
    for (id2, value2) in evalmap2.iter(){
        
        //if the evalmap1 has this curid value
        if let Some(value1) = evalmap1.get_mut(id2){
            
            //add this value to it
            *value1 += value2;
            
        }
        else{
            
            //otherwise create it with the evalmap1 value and put it in
            evalmap1.insert(*id2, *value2);
        }
        
        
    }
    
    
}


//given two evalmaps, get the total evalulation as a number
fn getevaluation(evalmap1: &HashMap< u32, f32> , evalmap2: &HashMap< u32, f32> ) -> f32{
    
    let mut totalvalue: f32 = 0.0;
    
    //for every id and value in evalmap1
    for (id1, value1) in evalmap1.iter(){
        
        //if evalmap2 also has this value
        if let Some(value2) = evalmap2.get(id1){
            
            totalvalue += value1 * value2;
            
        }
        
    }
    
    totalvalue
}




//dome
fn drawconvex(mut imageoutput: &mut Vec<Vec<(u8,u8,u8)>>, shape: &ConvexPolygon<f32>, pos: &Isometry2<f32>, leftedge: i16, topedge: i16, rgb:(u8,u8,u8)){
    
    
    //for each of the points
    for curpointnum in 0..shape.points().len(){
        
        let otherpointnum: usize;
        
        if (curpointnum == shape.points().len() -1){
            
            otherpointnum = 0;
            
        }
        else{
            
            otherpointnum = curpointnum + 1;
        }
        
        
        let startpoint = pos.transform_point( &shape.points()[curpointnum] );
        let endpoint = pos.transform_point( &shape.points()[otherpointnum] );
        
        
        
        let mut startx = startpoint.x as i16;
        let mut starty = startpoint.y as i16;
        
        let mut endx = endpoint.x as i16;
        let mut endy = endpoint.y as i16;
        
        
        drawline(&mut imageoutput, (startx, starty), (endx, endy)   , leftedge, topedge ,rgb);
        
        
        //println!("{:?}, {:?}, {:?}, {:?}", startpoint.x, startpoint.y, endpoint.x, endpoint.y);       
        
        
    }
    
    
    
}

//done
fn drawline(imageoutput: &mut Vec<Vec<(u8,u8,u8)>>, startxandy: (i16, i16), endxandy: ( i16, i16), leftedge: i16, topedge: i16, rgb:(u8,u8,u8)){
    
    
    let startx = startxandy.0 as i16;
    let starty = startxandy.1 as i16;
    
    let endx = endxandy.0 as i16;
    let endy = endxandy.1 as i16;
    
    let totalpoints = (endy - starty).abs() + (endx - startx).abs();
    
    
    for curpoint in 0..totalpoints{
        
        let totalx = endx - startx;
        let totaly = endy - starty;
        
        let curx = (startx as f32 + (totalx as f32 * (curpoint as f32/totalpoints as f32)) ) as i16;
        
        let cury = (starty as f32 + (totaly as f32 * (curpoint as f32/totalpoints as f32)) ) as i16;
        
        //make the curx and cury in range, and also, subtract the left and top edge
        
        let curx = makeinrange(0, imageoutput.len() as i16 -1      , curx - leftedge);
        let cury = makeinrange(0, imageoutput[0].len() as i16 -1   , cury - topedge);
        
        imageoutput[curx as usize ][cury as usize] = rgb;
        
    }
    
}

//done
fn makeinrange(minvalue: i16, maxvalue: i16, thevalue: i16) -> i16{
    
    if (thevalue > maxvalue){
        
        return(maxvalue)
    }
    
    if (thevalue < minvalue){
        
        return(minvalue)
    }
    
    thevalue
    
}