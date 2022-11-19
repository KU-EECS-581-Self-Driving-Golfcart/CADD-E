import React from 'react'
import { Link } from 'react-router-dom'

const Welcome = () => {
  return (
    <div className='flex flex-col items-center h-[100vh] justify-center'>
        <h1 className='text-6xl w-[750px] text-center mt-[-1em]'>Welcome to CADD-E!</h1>
        <h1 className='text-6xl w-[750px] text-center'>Select a Tee-Off Location to get Started:</h1>
        <div className='flex gap-16'>
            <div className='flex flex-col text-6xl text-white gap-7 mt-[1.8em]'>
                <Link to="/Main" state={{
                    state: { tee: "black"}
                }}><button className='bg-black p-4 rounded-2xl w-[400px]'>Black</button>
                </Link>
                <Link to="/Main" state={{
                    state: { tee: "bronze"}
                }}><button className='bg-[#CD7F32] p-4 rounded-2xl w-[400px]'>Bronze</button>
                </Link>
            </div>
            <div className='flex flex-col text-6xl text-white gap-7 mt-[1.8em]'>
                <Link to="/Main" state={{
                    state: { tee: "silver"}
                }}><button className='bg-[#C0C0C0] p-4 rounded-2xl w-[400px]'>Silver</button>
                </Link>
                <Link to="/Main" state={{
                    state: { tee: "gold"}
                }}><button className='bg-[#FFD700] p-4 rounded-2xl w-[400px]'>Gold</button>
                </Link>
            </div>
        </div>
    </div>
  )
}

export default Welcome