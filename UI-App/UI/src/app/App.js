import React from 'react'
import Main from './Main';
import Welcome from '../components/Welcome';
import { BrowserRouter, Routes, Route } from 'react-router-dom'

function App(props) {
  return (
    <div>
    <BrowserRouter>
      <Routes>
        <Route path="/" element={<Welcome />} />
        <Route path="/main" element={
          <Main 
            state={props.state}
            dispatch={props.dispatch}
          />
        }/>
       </Routes>
    </BrowserRouter>
    </div>
  );
}

export default App;
