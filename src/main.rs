
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


fn main() {
    println!("Hello, world!");
    
    
    let mut mypathfinding = Pathfinding::new();
    
    
    
    
    
    
    let points = [
    Point2::new(-3.0, -2.0),
    Point2::new(-3.0, 525.0),
    Point2::new(525.0, 525.0),
    Point2::new(525.0, -2.0),
    ];
    
    let theshape = ConvexPolygon::try_from_points(&points).expect("Convex hull computation failed.");
    let theisometry = Isometry2::new(Vector2::new(50.0, 50.0), 0.0);//std::f32::consts::FRAC_PI_2);
    
    let mut pathmap = HashMap::new();
    
    let mut entermap = HashMap::new();
    
    let mut exitmap = HashMap::new();
    

    let shape1 = mypathfinding.addshape( theshape, theisometry, pathmap, entermap, exitmap);
    
    
    
    
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


    //each agent has a value for each other agent
    //the value to be at and the value to move through

    //some objects you kinda dont need to move through
    //its not really about moving through them, its just about being at them

    //like lava and fire and ice and hard to move through mud and shit
    //are hard to move through and mean a more signficant disadvantage to move through
    //signficant amounts of them rather than insignificant amounts of them
    //but some objects you kinda only care about interacting with


    //what about a cost to move through, a cost to enter, and a cost to exit for all objects
    //would i still NEED a "goal" value


    //not really
    //instead of a "goal" value for the chest, the chest would have a high "enter" value
    //so you want to enter it, and it would have a high "exit" value until the chest was opened

    //and then when it was opened, the 

    //i can replace everything with a high "goal" instead with having a low, negative "enter" value
    //and then a low "exit" value
    //and then when the thing about the shape makes it valueless again, it loses that high value to enter
    //and high value to leave


    //the cost to walk somewhere increases as the intensity of the situation increases, and teh amount of shit to do increases





    
}

impl PathfindingShape{
    
    fn new(thepos: Isometry2<f32>, theshape: ConvexPolygon<f32>, pathmap: HashMap<u32, f32>, entermap: HashMap<u32, f32>, exitmap: HashMap<u32, f32> ) -> PathfindingShape{
        
        PathfindingShape{pos: thepos, shape: theshape, pathmap: pathmap, entermap: entermap, exitmap: exitmap}
    }
    
}


#[derive(Debug)]
pub struct PathfindingSegment{
    
    
    //the id of the two nodes this segment is between
    node1id: u32,
    node2id: u32,
    
    segment: Segment<f32>,
    
    leafid: DBVTLeafId,

    evalmap: HashMap<u32, f32>,
    
}

impl PathfindingSegment{
    
    fn new(thesegment: Segment<f32>, dbvtleafid: DBVTLeafId, node1id: u32, node2id: u32, evalmap: HashMap<u32, f32>) -> PathfindingSegment{
        
        PathfindingSegment{segment: thesegment, leafid: dbvtleafid, node1id: node1id, node2id: node2id, evalmap: evalmap}
    }

    
    //given an evaluation map of an agent, return the cost that following this segment will cost
    // (or gain, it can be a positive value when its a goal node)
    fn getvalue(&self, agentevalmap: &HashMap<u32, f32> ) -> f32{

        getevaluation(&self.evalmap, agentevalmap )

    }

    //given a nodeid of one of
    fn getothernode(&self, thisnodeid: u32) -> u32{

        if (thisnodeid == self.node1id){

            self.node2id
            
        }
        else if ( thisnodeid == self.node2id ) {

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


    
    
}

impl PathfindingNode{
    
    fn new( basepoint: Point2<f32>, linenormal: Vector2<f32> ) -> PathfindingNode{
        
        PathfindingNode{
            
            realpoint: basepoint,
            
            basepoint: basepoint,
            
            linenormal: linenormal,
            
            attachedsegmentids: HashSet::new(),
            
            
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
    
    
    //a hashmap of pathfinding shapes
    shapemap: HashMap<u32, PathfindingShape>,
    segmentmap: HashMap<u32, PathfindingSegment>,
    nodemap: HashMap<u32, PathfindingNode>,
    
    
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
            
            
            //a map between a node and all of the segments attached to it
            
            
            shapedbvt: DBVT::new(),
            
            connectiondbvt: DBVT::new(),
            
            realnodedbvt: DBVT::new(),
            
            
            linelengthpernode: 70.0,
            
            
            agentwidth: 55.0,
            
            
            shapesetmanager: ShapeSetManager::new(),
            
        }
    }
    
    
    fn createnode(&mut self, nodebase: Point2<f32>, nodenormal: Vector2<f32>, shapestobein: Vec<u32>,shapestobeout: Vec<u32> ) {
        
        
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
        
        for curshape in shapestobein{    
            
            nodeshapeset.insert(curshape);
        }
        
        for curshape in shapestobeout{
            
            nodeshapeset.remove(&curshape);
        }
        
        
        self.shapesetmanager.addnode(nodeid, nodeshapeset);
        
        
        //println!("{:?}", self.shapesetmanager);
        
        self.setconnections(nodeid);
    }
    
    
    //given a node
    //get the highest value path for it as a list of points to visit
    fn getpath(&self, nodeid: u32, evalmap: HashMap< u32, f32>) -> Vec< Point2<f32> >{
        
        //given a node, i want to get all the nodes that are attached to it
        //and i want to get all of the segments connected to that node
        

        //I NEED to figure out what the highest possible score of a node is
        //because the path value is going to start at zero
        //and its going to be kept adding to as it goes along new paths
        //and i should always stop the program when the lowest node score
        //is higher than the highest value path, because i know from then on
        //all the new goal nodes reached are going to be too high value for me to
        //go to
        //basically, i should have a list of goalnodes reached
        //that are the score when i arrived at them
        //and i should only consider them, and add them to the goal nodes reached set
        //if the value of that goalnode reached is higher than the pathvalue it takes to get to it


        //for each node


        //if the current pathvalue on reaching a node is less than zero, add it to the list of goalnodes reached
        let mut goalnodesreached: HeapWithValue = HeapWithValue::new();

        //this is the value that the path to the goalnode must have if i am to want to visit it
        let minimummotivatingvalue = -1.0;

        /*
        WAIT, IVE MADE A PRETTY IMPORTANT MISTAKE
        THE THING THAT MAKES A NODE WANT TO BE REACHED ISNT THE COST OF REACHING IT

        ITS THE VALUE FROM GETTING TO IT - THE COST OF REACHING IT

        like the GOAL of something is completely separate from the cost of getting to it

        like, a negative pathvalue doesnt make much sense

        the pathvalue should always be positive

        and then to determine if a node is a node i want to go to, it should be
        whether the evaluation for the shapeset that that node is in, is higher than
        the value of the shapeset that im currently in

        like its trying to reach better shapesets, and would be willing to go to better shapesets
        if the cost of going to them is lower than the difference between that shapesets value - this shapesets value


        */
        
        
        //the set of nodes i have already visited
        let mut visitednodes: HashSet<u32> = HashSet::new();
        
        
        let mut tovisit : HeapWithValue = HeapWithValue::new();
        
        
        
        //the list of points to visit
        let mut toreturn = Vec::new();
        
        
        
        //start the loop with the node passed in
        let curnodeid = nodeid;
        
        
        loop{
            
            //get the highest value node from set of nodes to visit
            let (curpathvalue, curnodeid) = tovisit.pop();
            
            //if this node has already been visited
            if (visitednodes.contains(&curnodeid) ){
                
                //i shouldnt do anything with this node
                //just go on to the next one
                
            }
            else
            {
                
                //add it to the list of nodes visited
                visitednodes.insert(curnodeid);
                
                
                //get this current node
                let curnode = self.nodemap.get(&curnodeid).unwrap();
                
                //get all the segments that are associated with it
                let associatedsegmentids: HashSet<u32> = curnode.get_segment_ids();
                
                //for each of the assocaited segments
                for cursegmentid in associatedsegmentids{

                    let cursegment = self.segmentmap.get(&cursegmentid).unwrap();

                    let othernodeid = cursegment.getothernode(curnodeid);


                    //make the node value to reach the current node value plus the value that will be given when
                    //calling the getvalue method on this segment with the evalmap of the agent passed in
                    //which will get what the value is with the segment evalmap + this agents evalmap
                    let newpathvalue = curpathvalue + cursegment.getvalue( &evalmap );

                    //ONLY RUN THIS IF THIS NEW PATH VALUE IS NOT ABOVE THE HIGHEST GOAL NODE VALUE
                    {

                        if (newpathvalue < minimummotivatingvalue){

                            goalnodesreached.insert(newpathvalue, othernodeid);
                        }

                        tovisit.insert( newpathvalue, othernodeid );
                    }

                    //if this current node is a goal node, add it to the set of goal nodes reached + the value
                    //that the goal node was reached at, so when theres no more nodes to visit anymore, we can
                    //return the highest value path to which goalnode
                    //(each goalnode can only be reached one highest-value way)
                    //(this is about which goalnode is the highest value - the pathvalue required to reach it)
                    
                    
                }
                
                
            }
            
            break;
            
            
            
            
            
            
        }
        
        
        
        
        
        
        
        toreturn
        
    }
    
    
    
    //TODO
    //THIS FUNCTION
    fn moveshape(&mut self, shapeid: u32){
        
        
    }
    
    
    //TODO
    
    //BE ABLE TO MOVE A SHAPE AND KEEP THE NODES ATTACHED IF VALID AND DISATTACHED IF NOT
    
    //IMPLEMENT THE NODEBOUNDINGS
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
                    
                    
                    self.connectiondbvt.remove(pathsegment.leafid);
                    
                    self.segmentmap.remove(&possibleintersectid);
                    
                    
                    
                    
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
                    let connectingslerps = 0;
                    
                    for curstep in 0..connectingslerps{
                        
                        let slerpnormal = curnormal.slerp(&nextnormal,     ((curstep + 1) as f32) / ((connectingslerps + 1) as f32) );
                        
                        
                        let nodeposition = point2;// + ( slerpnormal.into_inner() * self.agentwidth/2.0 ) * 1.05;
                        
                        
                        nodepostocreate.push( (nodeposition, slerpnormal.into_inner()) );
                        
                    }
                    
                }
            }
            
            
            //now create all of the nodes at those positions and add them to the collision world
            for (nodepos, nodenormal) in nodepostocreate{
                
                //create one node inside the shape and one node outside the shape for each node along the shape
                
                let mut shapestobein: Vec<u32> = Vec::new(); 
                let mut shapestobeout: Vec<u32> = Vec::new();
                shapestobein.push(shapeid);
                
                self.createnode( nodepos, nodenormal, shapestobein, shapestobeout);
                
                //nodescreated.push( ( self.createnode( &curnodepos), shapestobein, shapestobeout ) );
                
                let mut shapestobein: Vec<u32> = Vec::new(); 
                let mut shapestobeout: Vec<u32> = Vec::new();
                shapestobeout.push(shapeid);
                
                self.createnode( nodepos, nodenormal, shapestobein, shapestobeout);
                
                //nodescreated.push( ( self.createnode( &curnodepos), shapestobein, shapestobeout ) );
                
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
    
    
    //set the connections for a given node
    fn setconnections(&mut self, nodeid: u32){
        
        //get this node
        //actually, i have to get it later cuz i need it as mutable
        //let thisnode = self.nodemap.get(&nodeid).unwrap();
        
        //get all other nodes in the same shape set as this node
        let thenodes = self.shapesetmanager.getnodesinsameshapeset(&nodeid);
        
        //println!("the nodes in teh same set{:?}", self.shapesetmanager.getnodesinsameshapeset(&nodeid) );
        
        let thisnodepos = self.nodemap.get(&nodeid).unwrap().realpoint;
        
        use ncollide2d::bounding_volume::bounding_volume::HasBoundingVolume;
        
        //create a segment between every node in the same shape set
        for othernodeid in thenodes{
            
            //get the other node
            let othernode = self.nodemap.get_mut(&othernodeid).unwrap();
            
            
            let othernodepos = othernode.realpoint;
            
            
            let newsegment = Segment::new(thisnodepos, othernodepos);
            
            
            
            let mut collectedall: Vec<(u32)> = Vec::new();
            
            //create a bounding and grow it by agentwidth / 2
            let mut thebounding : AABB<f32> = newsegment.local_bounding_volume();
            thebounding.loosen(self.agentwidth / 2.0 );
            
            let mut thevisitor = BoundingVolumeInterferencesCollector::new( &thebounding, &mut collectedall);
            
            
            self.shapedbvt.visit( &mut thevisitor);
            
            
            
            let mut validconnection = true;
            
            
            for shapeid in collectedall{
                use ncollide2d::query::Proximity;
                
                //see if the shape and the segment are actually within proximity
                let thepathshape = self.shapemap.get(&shapeid).unwrap();
                
                let theproximity = proximity(
                    &Isometry2::identity(),
                    &newsegment, 
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
                
                //I SHOULDNT CREATE A SEGMENT FOR THIS NODE THAT ALREADY EXISTS
                
                let segmentid = self.totallines;
                self.totallines += 1;
                
                
                
                //create a bounding volume for the segment
                let mut thebounding: AABB<f32> = newsegment.local_bounding_volume();
                thebounding.loosen(self.agentwidth / 2.0 );
                
                //create a dbvt leaf for this node at this point
                let theleaf = DBVTLeaf::new( thebounding, segmentid);
                
                //add this node to the node dbvt
                let connectionleafid = self.connectiondbvt.insert(theleaf);



                //the total shapeset of all the shape shapesets combined
                let mut connectionevalmap: HashMap<u32, f32> = HashMap::new();

                //get the shapeset that both of the nodes are in (because theyre in the same shapeset)
                for curshapeid in self.shapesetmanager.getshapesetofnode(&nodeid){

                    //get the shape
                    let curshape = self.shapemap.get(&curshapeid).unwrap();

                    //add the shapesevalmap to the evalmap for this connection

                    addevalmaps(&mut connectionevalmap, &curshape.pathmap);


                }
                
                
                let pathsegment = PathfindingSegment::new(newsegment, connectionleafid, nodeid, othernodeid, connectionevalmap);
                
                self.segmentmap.insert(segmentid, pathsegment);
                
                
                
                othernode.addsegment(segmentid);
                
                let thisnode = self.nodemap.get_mut(&nodeid).unwrap();
                
                //for both the node and the other node, add this id to the list of segments its attached to
                thisnode.addsegment(segmentid);
                
                
                
                
            }
            
            
            
            
            
        }
        
        
        
        
        
    }
    
    
    
    //draws all of the shapes and all of the nodes and all of the connections
    //done
    fn drawstate(&self){
        
        let mut imageoutput: Vec<(Vec<(u8,u8,u8)>)>  = Vec::new();
        
        let leftedge = -100;
        let topedge = -100;
        let bottomedge = 1000;
        let rightedge = 1000;
        
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
    
    
    
    
    
    
    //every time a shape moves, check all the nodes inside it, and see if they are still inside that same shape
    //check what connection lines the shape now intersects with and destroy those
    //and all the nodes that moved along with the shape, check if their connection lines are still valid
    
    
    
    
    
    
    
    //i dont actually care what set of shapes a shape is in, only what sets of shapes nodes are in
    
    
    
    
    //get all the shapes the node is in
    
    
    
    
    
    
}



//this is a heap that has associated with it
struct HeapWithValue{
    
    //the list of the values and the nodes
    
    mainlist: BTreeMap< NotNan<f32> , HashSet<u32> >,
    
    
}


impl HeapWithValue{
    
    fn new() -> HeapWithValue{
        
        HeapWithValue{ mainlist: BTreeMap::new() }
        
        
    }
    
    //get the pathvalue of the highest cost node, and the id of the associated node
    fn pop(&mut self) -> ( f32 , u32 ){
        
        
        let ( key, _ ) = self.mainlist.last_key_value().unwrap();
        let key = key.clone();
        let nodeset = self.mainlist.get_mut(&key).unwrap();

        let pathvalue = key.into_inner();
        
        
        let mut maybereturnnodeid : Option<u32> = None;
        
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

            //return the pathvalue and the id of the node
            (pathvalue, returnnodeid)
        
        }
        else{

            panic!("the return node id didnt exist in thsi set of nodes");
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