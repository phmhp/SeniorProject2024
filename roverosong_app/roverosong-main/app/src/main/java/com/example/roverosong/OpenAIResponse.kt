package com.example.roverosong

data class OpenAIResponse(
    val choices: List<Choice>
)

data class Choice(
    val message: Message
)
