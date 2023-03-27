import { createSlice } from "@reduxjs/toolkit";

// Create the initial state of the counter
const initialState = {
    currentTee : 1,
    targetTee: 1,
    targetGreen: 1,
    teeOrGreen: 0
}

// Create the slice of the state
const teeInfoSlice = createSlice({
    name : 'teeInfo',
    initialState,
    reducers : {
        // Increment the counter by 1 when the increment action is dispatched
        incrementCurrent : (state) => {if(state.currentTee <9) state.currentTee = state.currentTee + 1;},
        incrementTarget : (state) => {if(state.targetTee<9) state.targetTee = state.targetTee + 1;},
        incrementGreen : (state) => {if(state.targetGreen<9) state.targetGreen = state.targetGreen+1;},
        setTeeOrGreen : (state) => {if(state.teeOrGreen == 0) state.teeOrGreen = 1; else {state.teeOrGreen = 0}}
    }
});

// Export the actions of the slice
export const {incrementCurrent, incrementTarget, incrementGreen, setTeeOrGreen} = teeInfoSlice.actions;
// Export the reducer of the slicel
export default teeInfoSlice.reducer;