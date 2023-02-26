import React from 'react'
import { Icon } from '@iconify/react';

const StopButton = (props) => {
  return (
    //Render the stop button that will be displayed once the Go button has been selected
    <button style={{display: props.isActive ? 'block' : 'none'}} onClick={props.stop} className='my-6 hidden'>
         <Icon icon="game-icons:stop-sign"  className='text-[275px] text-red-700 border-[#3E3B3B] drop-shadow-2xl'/>
    </button>
  )
}

export default StopButton