import React from 'react'

const Speedometer = (props) => {
  return (
    //Render the speedomter component that will be updated with the vehicles current speed
    <div className='flex flex-row justify-center border-[#3E3B3B] border-[3px] drop-shadow-2xl shadow-2xl border-solid rounded-xl py-16 mt-12 w-[350px] mt-[-1em]'>
        <h1 className='text-[#3E3B3B] text-5xl'>{props.speed}</h1>
        <h1 className='text-[#3E3B3B] text-5xl ml-2'>mph</h1>
    </div>
  )
}

export default Speedometer