import { Icon } from '@iconify/react';

const TeeCounter = (props) => {
    return (
    //Render the tee counter that will display the current tee location
    <div className='flex text-6xl justify-center bg-[#3E3B3B] text-white w-[200px] py-2 rounded-b-2xl shadow-xl'>
        <Icon icon="game-icons:golf-tee" className=''/>
        <h2 className='ml-2 tee'>{props.currentTee}</h2>
    </div>
    )
}

export default TeeCounter