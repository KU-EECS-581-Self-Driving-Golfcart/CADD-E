import React from 'react'
import Main from './components/Main';
import Welcome from './components/Welcome';
import { BrowserRouter, Routes, Route } from 'react-router-dom'

function App() {
  return (
    <div>
    <BrowserRouter>
      <Routes>
        <Route path="/" element={<Welcome />} />
        <Route path="/main" element={<Main/>} />
       </Routes>
    </BrowserRouter>
    </div>
  );
}

export default App;
