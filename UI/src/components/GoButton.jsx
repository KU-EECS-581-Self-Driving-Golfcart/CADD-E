import React from 'react'
import { Icon } from '@iconify/react';

const GoButton = (props) => {
  return (
    <button style={{display: props.isActive ? 'none' : 'block'}} className='my-10' onClick={props.go}>
        <div className="flex bg-green-700 aspect-square shrink-0 rounded-full grow-0 w-[260px] h-[260px] justify-center items-center flex-col shadow-2xl drop-shadow-2xl">
        <Icon icon="mdi:golf-cart" className='text-white text-9xl mt-[1rem]'/>
        <Icon icon="file-icons:go" className='text-white text-9xl mt-[-.35em] mr-[.2em]'/>
        </div>
  </button>
  )
}

export default GoButton