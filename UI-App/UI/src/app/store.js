import { configureStore } from '@reduxjs/toolkit'
import teeInfoReducer from "../components/teeInfoSlice"

const store = configureStore({
    reducer: {
        teeInfo: teeInfoReducer
    }
})

export default store