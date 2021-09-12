

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