import React from 'react';
import ReactDOM from 'react-dom/client';
import './index.css';
import App from './app/App';
import store from './app/store.js'
import { Provider } from 'react-redux'
import EventEmitter2 from 'eventemitter2'
window.EventEmitter2 = EventEmitter2.EventEmitter2

const root = ReactDOM.createRoot(document.getElementById('root'));

root.render(
    <Provider store={store}>
        <App />
    </Provider>
);

