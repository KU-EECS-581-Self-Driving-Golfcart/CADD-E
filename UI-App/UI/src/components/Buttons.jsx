import React, {useEffect, useState} from 'react';
import {useSelector, useDispatch} from 'react-redux';
import {pathOne, pathTwo, pathThree} from '../paths.js'
import {tee1,tee2,tee3,tee4,tee5,tee6,tee7,tee8,tee9} from '../Data/tees';
import { setTeeOrGreen } from './teeInfoSlice';
import { useLocation } from 'react-router-dom'
import ROSLIB from 'roslib'
import {teePub} from '../app/App.js'

export const selectFilter = state => state.filter;

//Buttons Componenet will be used to render the "Next Tee" & "Next Green" Buttons
const Buttons = (props) => {
  const tee = useLocation().state.state.tee; 
  let teeType = "";

  const teeOrGreen = useSelector(state => state.teeInfo.teeOrGreen);
  if(tee === "black") teeType = "B";
  if(tee === "bronze") teeType = "R";
  if(tee === "silver") teeType = "S";
  if(tee === "gold")  teeType = "G";

  const dispatch = useDispatch();
  const targetTee = useSelector(state => state.teeInfo.targetTee);
  const targetGreen = useSelector(state => state.teeInfo.targetGreen);

  const [data, setData] = useState({}) //Used to inform target locations 
  //incTee is called onClick if the Button is a "Next Tee"
  function incTee() {
    console.log(props.validGreenTee);
    if(props.validNextTee && !props.isGo) //Ensure that incramenting the tee is valid
    {
      //Set proper state variables
      props.setValidNextTee(false); 
      props.setValidGreenTee(true);
      props.setValidTarget(true);


      //Set corresponding state positions depending on the tee that is being targeted
      if(targetTee===1) {
        if(props.tee === "black") props.setNextPosition(tee2.blackPos);
        if(props.tee === "bronze") props.setNextPosition(tee2.bronzePos);
        if(props.tee === "silver") props.setNextPosition(tee2.silverPos);
        if(props.tee === "gold") props.setNextPosition(tee2.goldPos);
      }
      if(targetTee===2) {
        if(props.tee === "black") props.setNextPosition(tee3.blackPos);
        if(props.tee === "bronze") props.setNextPosition(tee3.bronzePos);
        if(props.tee === "silver") props.setNextPosition(tee3.silverPos);
        if(props.tee === "gold") props.setNextPosition(tee3.goldPos);
      }
      if(targetTee===3) {
        if(props.tee === "black") props.setNextPosition(tee4.blackPos);
        if(props.tee === "bronze") props.setNextPosition(tee4.bronzePos);
        if(props.tee === "silver") props.setNextPosition(tee4.silverPos);
        if(props.tee === "gold") props.setNextPosition(tee4.goldPos);
      }
      if(targetTee===4) {
        if(props.tee === "black") props.setNextPosition(tee5.blackPos);
        if(props.tee === "bronze") props.setNextPosition(tee5.bronzePos);
        if(props.tee === "silver") props.setNextPosition(tee5.silverPos);
        if(props.tee === "gold") props.setNextPosition(tee5.goldPos);
      }
      if(targetTee===5) {
        if(props.tee === "black") props.setNextPosition(tee6.blackPos);
        if(props.tee === "bronze") props.setNextPosition(tee6.bronzePos);
        if(props.tee === "silver") props.setNextPosition(tee6.silverPos);
        if(props.tee === "gold") props.setNextPosition(tee6.goldPos);
      }
      if(targetTee===6) {
        if(props.tee === "black") props.setNextPosition(tee7.blackPos);
        if(props.tee === "bronze") props.setNextPosition(tee7.bronzePos);
        if(props.tee === "silver") props.setNextPosition(tee7.silverPos);
        if(props.tee === "gold") props.setNextPosition(tee7.goldPos);
      }
      if(targetTee===7) {
        if(props.tee === "black") props.setNextPosition(tee8.blackPos);
        if(props.tee === "bronze") props.setNextPosition(tee8.bronzePos);
        if(props.tee === "silver") props.setNextPosition(tee8.silverPos);
        if(props.tee === "gold") props.setNextPosition(tee8.goldPos);
      }
      if(targetTee===8) {
        if(props.tee === "black") props.setNextPosition(tee9.blackPos);
        if(props.tee === "bronze") props.setNextPosition(tee9.bronzePos);
        if(props.tee === "silver") props.setNextPosition(tee9.silverPos);
        if(props.tee === "gold") props.setNextPosition(tee9.goldPos);
      }
      // props.setRoutePos(pathTwo);
      dispatch(setTeeOrGreen) //Set 0 to signal that the current target is a Tee
      // let obj = {
      //   teeBox: targetTee+1,
      //   type: teeType
      // }
      var teeInfo = new ROSLIB.Message({
        data: targetTee+1 + teeType 
      });
      teePub.publish(teeInfo);
      // console.log(obj);
      //   fetch("/TargetLoc", {
      //     'method': "POST",
      //     headers: {
      //       "Content-Type":"application/json",
      //     },
      //     body:JSON.stringify(obj)
      //   }).then(
      //     res => res.json()
      // ).then(response => response.json())
      // .catch(error => console.log(error))
    }
  }

  //incTee is called onClick if the Button is a "Next Tee"
  function incGreen() {
    if(props.validGreenTee && !props.isGo) //Ensure that incramenting the green is valid
    {
      //Set proper state variables
      props.setValidGreenTee(false);
      props.setValidNextTee(true);
      props.setValidTarget(true)

      //Set corresponding state positions depending on the green location that is being targeted
      if(targetGreen===1) props.setNextPosition(tee1.greenPos);
      if(targetGreen===2) props.setNextPosition(tee2.greenPos);
      if(targetGreen===3) props.setNextPosition(tee3.greenPos);
      if(targetGreen===4) props.setNextPosition(tee4.greenPos);
      if(targetGreen===5) props.setNextPosition(tee5.greenPos);
      if(targetGreen===6) props.setNextPosition(tee6.greenPos);
      if(targetGreen===7) props.setNextPosition(tee7.greenPos);
      if(targetGreen===8) props.setNextPosition(tee8.greenPos);
      if(targetGreen===8) props.setNextPosition(tee9.greenPos);
      dispatch(setTeeOrGreen);

      // var cmdVel = new ROSLIB.Topic({
      //   ros : ros,
      //   name : '/teeInfo',
      //   messageType : 'std_msgs/String'
      // });
    
      // var twist = new ROSLIB.Message({
      //   data: "Hello, world!"
      //     }    );
      // cmdVel.publish(twist);


      // let obj = {
      //   teeBox: targetTee,
      //   type: 'H'
      // }
      var teeInfo = new ROSLIB.Message({
        data: targetTee + 'H' 
      });
      teePub.publish(teeInfo);
      // console.log(obj);
      //   fetch("/TargetLoc", {
      //     'method': "POST",
      //     headers: {
      //       "Content-Type":"application/json",
      //     },
      //     body:JSON.stringify(obj)
      //   }).then(
      //     res => res.json()
      // ).then(response => response.json())
      // .catch(error => console.log(error))
    }
  }

  let isTee = false;
  if(props.name === "Next Teebox") isTee = true;

  return (
    <div>
      <input style={{cursor: "pointer"}} onClick={isTee ? incTee : incGreen} type="submit" className='bg-[#3E3B3B] text-white py-4 w-[275px] rounded-xl shadow-lg text-3xl' value={props.name}></input>
    </div>
  )
}

export default Buttons