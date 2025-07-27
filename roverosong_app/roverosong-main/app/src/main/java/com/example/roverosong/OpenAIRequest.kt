package com.example.roverosong

data class OpenAIRequest(
    val model: String = "gpt-3.5-turbo",
    val messages: List<Message>,
    val max_tokens: Int = 200
)

data class Message(
    val role: String,
    val content: String
)
