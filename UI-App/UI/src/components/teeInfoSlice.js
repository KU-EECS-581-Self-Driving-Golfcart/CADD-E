import { createSlice } from "@reduxjs/toolkit";

// Create the initial state of the counter
const initialState = {
    currentTee : 1
}

// Create the slice of the state
const teeInfoSlice = createSlice({
    // The name of the slice
    name : 'teeInfo',
    // The initial state of the slice
    initialState,
    // The reducers of the slice
    reducers : {
        // Increment the counter by 1 when the increment action is dispatched
        increment : (state) => {
            if(state.currentTee <9) state.currentTee = state.currentTee + 1;
        },
    }
});

// Export the actions of the slice
export const {increment} = teeInfoSlice.actions;
// Export the reducer of the slicel
export default teeInfoSlice.reducer;